"""Execute a planned trajectory on the robot through the RB-Y1 driver.

The path is the driver's own follow_joint_trajectory action, not MoveIt and not
ros2_control. The driver streams each waypoint to the SDK with the gap to the
previous one as its minimum time, so the trajectory's timestamps *are* the
speed. That gives the caller direct control: pick a duration, retime to it.

Two driver behaviours shape this module:

* It drops every stream after 60 s without a command (check_stream_safety), and
  a trajectory whose stream has gone is skipped *while the action still reports
  success*. So the trajectory is resampled to short, even steps, and completion
  is judged from measured joints, never from the action result alone.
* It rejects the action unless stream control is on. So the stream is enabled
  immediately before sending.
"""

import math
import time

from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from rby1_msgs.srv import SetTrajectoryImpedance
import numpy as np
import rclpy
from scipy.interpolate import CubicHermiteSpline, CubicSpline
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rby1_cumotion.bringup.driver import Driver
from rby1_cumotion.model import origin

# Well inside the driver's 1 s stream timeout, and fine enough that the SDK's
# per-segment interpolation follows the spline closely.
DEFAULT_STEP = 0.05
MAX_STEP = 0.5
# Seconds of the final pose repeated after the motion. The driver reports the
# action done as soon as it has *sent* the last waypoint, while the arm is still
# catching up; switching the stream off then cuts the motion short. Measured on
# the simulator at 60-70% of the velocity limit: 2 of 6 moves stopped 0.03-0.055
# rad short and never converged. Holding keeps the stream fed while it settles.
DEFAULT_HOLD = 0.5
# The driver's stream_control, switched off and on again within ~0.1 s, can hand out
# a stream that is already expired: the next trajectory never reaches the robot
# (measured: 2 of 3 at 0.05 s, none at 0.3 s). Targets sent back to back hit this.
STREAM_REOPEN_GAP = 0.5
# How the driver's stream_control answer begins when it opened a channel asked for.
OPENED = 'Stream channels opened'


def stream_channels(joints):
    """The driver's stream channels (stream_control's parameters) these joints go out on."""
    prefixes = {'arm': ('right_arm_', 'left_arm_'), 'torso': ('torso_',)}
    return tuple(channel for channel, starts in prefixes.items()
                 if any(name.startswith(starts) for name in joints))
# The parts set_trajectory_impedance takes, in its order.
IMPEDANCE_PARTS = ('torso', 'right_arm', 'left_arm')


def impedance_request(impedance, group_joints, robot_joints):
    """set_trajectory_impedance for the group's parts: impedance if enabled, else position.

    The service sets all three parts at once; parts outside the group go to position.
    """
    request = SetTrajectoryImpedance.Request()
    for part in IMPEDANCE_PARTS:
        prefix = part + '_'
        on = bool(impedance['enabled']) and any(name.startswith(prefix) for name in group_joints)
        request.state.append(on)
        count = sum(1 for name in robot_joints if name.startswith(prefix))
        setattr(request, f'{part}_stiffness', [float(impedance['stiffness'])] * count if on else [])
    request.damping_ratio = [float(impedance['damping_ratio'])]
    request.torque_limit = [float(impedance['torque_limit'])]
    return request


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp(t):
    # Round the total first: splitting first turns 1.9999999999 into 1 s + 1e9 ns,
    # wrapped to 1.0 s -- a step back in time that the driver sleeps through.
    total = int(round(t * 1e9))
    return Duration(sec=total // 1_000_000_000, nanosec=total % 1_000_000_000)


class ToolChain:
    """Forward kinematics of one frame, along its chain only (fast enough to sample a path)."""

    def __init__(self, root, tip):
        by_child = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
        chain, link = [], tip
        while link in by_child:
            joint = by_child[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        if not chain:
            raise ValueError(f'{tip} is not below any joint of the model')
        self.steps = []
        for joint in reversed(chain):
            axis = None
            if joint.get('type') in ('revolute', 'continuous', 'prismatic'):
                axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
                axis /= np.linalg.norm(axis)
            self.steps.append((origin(joint.find('origin')), joint.get('type'), joint.get('name'), axis))

    def pose(self, positions):
        pose = np.eye(4)
        for offset, kind, name, axis in self.steps:
            pose = pose @ offset
            if axis is None:
                continue
            q = positions[name]
            delta = np.eye(4)
            if kind == 'prismatic':
                delta[:3, 3] = axis * q
            else:
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                delta[:3, :3] = np.eye(3) + math.sin(q) * skew + (1 - math.cos(q)) * skew @ skew
            pose = pose @ delta
        return pose


def _unit_spline(trajectory):
    times = np.array([seconds(point.time_from_start) for point in trajectory.points])
    positions = np.array([point.positions for point in trajectory.points], dtype=float)
    if len(times) < 2 or times[-1] <= 0 or np.any(np.diff(times) <= 0):
        raise ValueError('planned trajectory needs strictly increasing timestamps')
    return CubicSpline((times - times[0]) / (times[-1] - times[0]), positions, axis=0,
                       bc_type='clamped'), positions


def motion_duration(trajectory, chain, fixed, limits, linear_limit, angular_limit,
                    minimum_time, samples=120):
    """How long a move must take so the tool and every joint stay within their limits.

    Retiming keeps the path and scales time, so every speed along it is
    inversely proportional to the duration. One pass at a duration of 1 s gives
    each limit's shortest allowed duration; the move takes the longest of them
    and never less than `minimum_time`. Returns (seconds, {limit: seconds}).
    """
    spline, _ = _unit_spline(trajectory)
    names = list(trajectory.joint_names)
    t = np.linspace(0.0, 1.0, samples)
    q, qd = spline(t), spline(t, 1)
    joint = max(np.abs(qd[:, i]).max() / limits[name]['velocity'] for i, name in enumerate(names))
    poses = [chain.pose({**fixed, **dict(zip(names, row))}) for row in q]
    dt = t[1] - t[0]
    linear = max(np.linalg.norm(b[:3, 3] - a[:3, 3]) for a, b in zip(poses, poses[1:])) / dt
    angular = max(
        math.acos(max(-1.0, min(1.0, (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2)))
        for a, b in zip(poses, poses[1:])) / dt
    needed = {
        'minimum_time': float(minimum_time),
        'linear_velocity_limit': linear / linear_limit,
        'angular_velocity_limit': angular / angular_limit,
        # 2% over the sampled peak: retime() checks its own, finer, samples.
        'joint_velocity_limits': joint * 1.02,
    }
    return max(needed.values()), needed


def retime(trajectory, duration, step, limits):
    """Stretch a planned trajectory to `duration`, resampled every `step` seconds.

    The path is kept; only its timing changes. A clamped cubic spline through
    the planned waypoints starts and ends at rest. Raises ValueError when the
    requested duration would exceed a joint's velocity limit, naming the
    shortest duration that would not.
    """
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('duration must be a positive number of seconds')
    if not 0.001 <= step <= MAX_STEP:
        raise ValueError(f'step must be in [0.001, {MAX_STEP}] s')
    names = list(trajectory.joint_names)
    times = np.array([seconds(point.time_from_start) for point in trajectory.points])
    positions = np.array([point.positions for point in trajectory.points], dtype=float)
    if len(times) < 2 or times[-1] <= 0 or np.any(np.diff(times) <= 0):
        raise ValueError('planned trajectory needs strictly increasing timestamps')
    scaled = (times - times[0]) * (duration / (times[-1] - times[0]))
    spline = CubicSpline(scaled, positions, axis=0, bc_type='clamped')
    count = max(2, math.ceil(duration / step) + 1)
    samples = np.linspace(0.0, duration, count)
    q = spline(samples)
    qd = spline(samples, 1)
    # Pin the ends exactly: the spline passes through them, but not bit-exactly.
    q[0], q[-1] = positions[0], positions[-1]
    qd[0] = qd[-1] = 0.0
    worst, slowest = 0.0, None
    for column, name in enumerate(names):
        limit = limits[name]
        low, high = q[:, column].min(), q[:, column].max()
        if low < limit['lower'] - 1e-9 or high > limit['upper'] + 1e-9:
            raise ValueError(f'{name} would leave its limits [{limit["lower"]:.3f}, '
                             f'{limit["upper"]:.3f}] between waypoints; plan again')
        ratio = np.abs(qd[:, column]).max() / limit['velocity']
        if ratio > worst:
            worst, slowest = ratio, name
    if worst > 1.0:
        raise ValueError(f'{duration:.2f} s is too fast: {slowest} would reach {worst:.2f}x '
                         f'its velocity limit. Use at least {duration * worst:.2f} s.')
    result = JointTrajectory(joint_names=names)
    for t, position, velocity in zip(samples, q, qd):
        result.points.append(JointTrajectoryPoint(
            positions=position.tolist(), velocities=velocity.tolist(), time_from_start=stamp(t)))
    return result, worst


def times_of(trajectory):
    return np.array([seconds(point.time_from_start) for point in trajectory.points])


def state_at(trajectory, t):
    """Positions and velocities of a timed trajectory at time `t` (clamped to its span)."""
    times = times_of(trajectory)
    positions = np.array([p.positions for p in trajectory.points], dtype=float)
    velocities = np.array([p.velocities if len(p.velocities) == positions.shape[1]
                           else [0.0] * positions.shape[1] for p in trajectory.points], dtype=float)
    t = min(max(t, times[0]), times[-1])
    spline = CubicHermiteSpline(times, positions, velocities, axis=0)
    return spline(t), spline(t, 1)


def ahead(trajectory, t_now, horizon):
    """Waypoint positions timed in (t_now, t_now + horizon]: what the robot is about to do."""
    times = times_of(trajectory)
    rows = [p.positions for p, t in zip(trajectory.points, times) if t_now < t <= t_now + horizon]
    return np.array(rows, dtype=float).reshape(-1, len(trajectory.joint_names))


def splice(current, t_now, t_join, path, limits, step, floor=0.0):
    """A trajectory that replaces `current` while it runs, without a jolt.

    It starts where the driver is now (`t_now` in `current`, the last waypoint it
    sent), follows `current` unchanged up to `t_join` -- the time the robot will
    have reached while the replacement is computed and sent -- and then follows
    `path`, a replanned list of joint positions that starts at `current`'s state
    at `t_join`. That part is timed with a spline that begins with `current`'s
    velocity there and ends at rest, taking at least `floor` seconds so a detour
    is not faster than the move it replaces, and no less than the joint velocity
    limits need. Times in the result count from `t_now`.
    """
    names = list(current.joint_names)
    path = np.asarray(path, dtype=float)
    q_join, v_join = state_at(current, t_join)
    if np.abs(path[0] - q_join).max() > 1e-3:
        raise ValueError('replanned path does not start at the trajectory state it replaces')
    # Keep what the driver is about to send until the join.
    lead_times = np.arange(t_now + step, t_join - 1e-9, step)
    result = JointTrajectory(joint_names=names)
    for t in lead_times:
        q, v = state_at(current, t)
        result.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=v.tolist(),
                                                  time_from_start=stamp(t - t_now)))
    # Arc-length parameter of the new path, then time it.
    gaps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    keep = np.concatenate([[True], gaps > 1e-9])
    path = path[keep]
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    if arc[-1] < 1e-9:
        raise ValueError('replanned path does not move')
    velocity = {name: limits[name]['velocity'] for name in names}
    duration = max(floor, step * 2)
    for _ in range(40):
        spline = CubicSpline(arc / arc[-1] * duration, path, axis=0,
                             bc_type=((1, v_join), (1, np.zeros(len(names)))))
        samples = np.linspace(0.0, duration, max(2, math.ceil(duration / step) + 1))
        qd = spline(samples, 1)
        worst = max(np.abs(qd[:, i]).max() / velocity[name] for i, name in enumerate(names))
        if worst <= 1.0:
            break
        duration *= min(2.0, worst * 1.05)
    else:
        raise ValueError('could not time the replanned path within the joint velocity limits')
    q = spline(samples)
    q[-1] = path[-1]
    qd[-1] = 0.0
    offset = t_join - t_now
    for t, position, speed in zip(samples, q, qd):
        result.points.append(JointTrajectoryPoint(positions=position.tolist(), velocities=speed.tolist(),
                                                  time_from_start=stamp(offset + t)))
    return result


def slow_to_stop(current, t_now, stop_time, hold, step):
    """`current` from where the driver is (`t_now`), braked to rest on its own path.

    The path stays; only its clock slows. The rate at which `current`'s time
    advances falls linearly from 1 to 0 over `stop_time`, so the arm comes to rest
    `stop_time / 2` further along -- at `s_stop` in `current`'s time -- and holds
    there for `hold` s, long enough to keep the driver's stream busy while waiting.
    Times in the result count from `t_now`. Returns (trajectory, s_stop).
    """
    end = times_of(current)[-1]
    s_stop = min(t_now + stop_time / 2, end)
    result = JointTrajectory(joint_names=list(current.joint_names))
    count = max(1, math.ceil(stop_time / step))
    for i in range(1, count + 1):
        tau = stop_time * i / count
        rate = 1.0 - tau / stop_time
        q, v = state_at(current, min(t_now + tau - tau * tau / (2 * stop_time), end))
        result.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=(v * rate).tolist(),
                                                  time_from_start=stamp(tau)))
    final = state_at(current, s_stop)[0].tolist()
    rest = [0.0] * len(final)
    result.points[-1] = JointTrajectoryPoint(positions=final, velocities=rest,
                                             time_from_start=stamp(stop_time))
    for i in range(1, max(1, math.ceil(hold / step)) + 1):
        result.points.append(JointTrajectoryPoint(positions=final, velocities=rest,
                                                  time_from_start=stamp(stop_time + step * i)))
    return result, s_stop


def back_off(current, s_from, s_to, hold, step, start=None, limits=None, share=0.5):
    """From rest at `s_from` in `current`'s time, back along its path to `s_to` and hold.

    Rest to rest on a smoothstep clock. Without `limits` it is never faster along
    the path than `current` was planned to go; with them, as fast as keeps every
    joint within `share` of its velocity limit -- getting out of the way of
    something coming at the arm should not wait for a slow plan's pace. `start`
    is an optional braking trajectory to continue from (it ends at `s_from`).
    """
    result = JointTrajectory(joint_names=list(current.joint_names),
                             points=list(start.points) if start is not None else [])
    t0 = seconds(result.points[-1].time_from_start) if result.points else 0.0
    distance = s_to - s_from
    rate = 1.0  # the fastest the clock may run
    if limits is not None and abs(distance) > 1e-9:
        along = np.array([np.abs(state_at(current, s)[1]) for s in np.linspace(s_from, s_to, 20)]).max(axis=0)
        caps = [share * limits[name]['velocity'] / speed
                for name, speed in zip(current.joint_names, along) if speed > 1e-9]
        rate = max(1.0, min(caps)) if caps else 1.0
    duration = max(0.3, 1.5 * abs(distance) / rate)  # smoothstep peaks at 1.5 |d| / T
    count = max(1, math.ceil(duration / step))
    for i in range(1, count + 1):
        u = i / count
        s = s_from + distance * (3 * u * u - 2 * u ** 3)
        rate = distance * 6 * u * (1 - u) / duration
        q, v = state_at(current, s)
        result.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=(v * rate).tolist(),
                                                  time_from_start=stamp(t0 + duration * u)))
    final = result.points[-1].positions
    for i in range(1, max(1, math.ceil(hold / step)) + 1):
        result.points.append(JointTrajectoryPoint(positions=list(final), velocities=[0.0] * len(final),
                                                  time_from_start=stamp(t0 + duration + step * i)))
    return result


def resume_from(current, s_stop, end, ramp_time, step):
    """The rest of `current` after `s_stop` (up to `end`), started from rest.

    The reverse of slow_to_stop: the clock speeds up from 0 to 1 over
    `ramp_time`, then `current` runs as planned. Times count from now; the first
    point is `step` ahead. Already at `end`: just that pose.
    """
    result = JointTrajectory(joint_names=list(current.joint_names))
    if s_stop >= end - 1e-6:
        q = state_at(current, end)[0]
        result.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=[0.0] * len(q),
                                                  time_from_start=stamp(step)))
        return result
    index = 0
    while True:
        index += 1
        tau = step * index
        s = s_stop + (tau * tau / (2 * ramp_time) if tau < ramp_time else ramp_time / 2 + tau - ramp_time)
        rate = min(tau / ramp_time, 1.0)
        if s >= end:
            q = state_at(current, end)[0]
            result.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=[0.0] * len(q),
                                                      time_from_start=stamp(tau)))
            return result
        q, v = state_at(current, s)
        result.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=(v * rate).tolist(),
                                                  time_from_start=stamp(tau)))


def with_hold(trajectory, hold, step):
    """Append `hold` seconds of the final pose, at rest, in `step` increments."""
    if hold < 0 or not math.isfinite(hold):
        raise ValueError('hold must be a non-negative number of seconds')
    if hold == 0:
        return trajectory
    end = seconds(trajectory.points[-1].time_from_start)
    final = list(trajectory.points[-1].positions)
    rest = [0.0] * len(final)
    count = max(1, math.ceil(hold / step))
    held = JointTrajectory(joint_names=list(trajectory.joint_names),
                           points=list(trajectory.points))
    for i in range(1, count + 1):
        held.points.append(JointTrajectoryPoint(
            positions=final, velocities=rest, time_from_start=stamp(end + hold * i / count)))
    return held


class DriverExecutor:
    """Sends retimed trajectories to one driver and confirms them from joint states."""

    def __init__(self, node, namespace='rby1', endpoint_tolerance=0.03,
                 hold=DEFAULT_HOLD, step=DEFAULT_STEP):
        self.node = node
        self.driver = Driver(node, namespace)
        self.endpoint_tolerance = endpoint_tolerance
        self.hold = hold
        self.step = step
        self.client = None
        self.goal_handle = None
        self.stream_closed = 0.0
        # Set by the node: the set_trajectory_impedance request it wants (or None),
        # and the driver's stream channels the group's joints go out on.
        self.impedance = lambda: None
        self.channels = ('arm',)
        self.applied_impedance = None
        self.owned = []  # the channels this executor opened, to close when done

    def connect(self):
        self.client = self.driver.action(FollowJointTrajectory, 'follow_joint_trajectory')

    def begin(self, trajectory):
        """Start `trajectory` (with the final hold) and return; see progress/replace/finish."""
        if self.client is None:
            self.connect()
        self.goal_handle = self.result_future = None
        self.progress = 0.0
        self.apply_impedance()
        self.open_stream()
        self.send(trajectory)

    def open_stream(self):
        """Open the group's stream channels. One that was open already is someone
        else's (a user's, another node's) and stays open when this executor is done:
        the driver's answer says which it opened."""
        time.sleep(max(0.0, self.stream_closed + STREAM_REOPEN_GAP - time.monotonic()))
        self.owned = [channel for channel in self.channels
                      if self.driver.switch('stream_control', True, channel).message.startswith(OPENED)]

    def close_stream(self):
        """Close the stream channels open_stream opened."""
        if not self.owned:
            return
        try:
            self.driver.switch('stream_control', False, ','.join(self.owned))
        except Exception as error:  # the driver may already have dropped it
            self.node.get_logger().warn(f'stream_control off: {error}')
        self.stream_closed = time.monotonic()
        self.owned = []

    def apply_impedance(self):
        """Hand the driver the impedance setting, when it changed since the last move."""
        request = self.impedance()
        if request is None:
            return
        key = (tuple(request.state), tuple(request.torso_stiffness), tuple(request.right_arm_stiffness),
               tuple(request.left_arm_stiffness), tuple(request.damping_ratio), tuple(request.torque_limit))
        if key == self.applied_impedance or (self.applied_impedance is None and not any(request.state)):
            return  # unchanged, or never enabled: the driver's own setting stands
        self.driver.call('set_trajectory_impedance', SetTrajectoryImpedance, request)
        self.applied_impedance = key
        parts = [part for part, on in zip(IMPEDANCE_PARTS, request.state) if on]
        self.node.get_logger().info(f'joint impedance for {parts}' if parts else 'joint position for every part')

    def send(self, trajectory):
        self.sent = with_hold(trajectory, self.hold, self.step)
        self.token = getattr(self, 'token', 0) + 1
        token = self.token
        goal = FollowJointTrajectory.Goal(trajectory=self.sent)
        handle = self.driver.wait(self.client.send_goal_async(
            goal, feedback_callback=lambda message: self.on_feedback(message, token)), 10.0)
        if not handle.accepted:
            raise RuntimeError('Driver rejected the trajectory -- ros2_control may be holding the '
                               'robot (hardware_control), or its start is too far from where the '
                               'robot is (the driver log says which)')
        self.goal_handle = handle
        self.result_future = handle.get_result_async()
        self.progress, self.progress_at = 0.0, time.monotonic()

    def on_feedback(self, message, token):
        # Feedback of a replaced goal can still arrive; only the running one counts.
        if token == self.token:
            self.progress = seconds(message.feedback.desired.time_from_start)
            self.progress_at = time.monotonic()

    def now(self):
        """Where the driver is in the running trajectory at this moment.

        Feedback arrives once per waypoint and only while the node spins, so after
        a long computation it lags: take what has arrived, then add the time since
        the last one (at most one waypoint's worth, which is all it can be off by).
        """
        for _ in range(20):  # drain what queued up while the node was not spinning
            rclpy.spin_once(self.node, timeout_sec=0.0)
        since = time.monotonic() - getattr(self, 'progress_at', time.monotonic())
        return self.progress + min(max(since, 0.0), self.step)

    def replace(self, trajectory):
        """Hand the driver a new trajectory while one runs; it takes over without a stop."""
        self.send(trajectory)

    def done(self):
        return self.result_future is not None and self.result_future.done()

    def motion_end(self):
        """Time in the running trajectory where its motion ends and the hold begins."""
        return seconds(self.sent.points[-1].time_from_start) - self.hold

    def finish(self, trajectory, snapshot):
        """Wait for the running trajectory, then check the arm really got there."""
        try:
            duration = seconds(self.sent.points[-1].time_from_start)
            result = self.driver.wait(self.result_future, duration + 30.0)
            self.goal_handle = None
            code = result.result.error_code
            if code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(f'Driver aborted the trajectory ({code}): '
                                   f'{result.result.error_string}')
        finally:
            self.stop()
        self.confirm(trajectory, snapshot)

    def stop(self):
        """Cancel whatever runs and close the stream channels this executor opened."""
        self.cancel()
        self.close_stream()

    def execute(self, trajectory, snapshot):
        """Run `trajectory` to the end; `snapshot()` must return current measured positions."""
        try:
            self.begin(trajectory)
        except Exception:
            self.stop()
            raise
        self.finish(trajectory, snapshot)

    def confirm(self, trajectory, snapshot, settle=3.0):
        """The action can report success after the stream silently dropped.

        Joints get `settle` seconds to converge after the last waypoint.
        """
        final = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        deadline = time.monotonic() + settle
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            try:
                current = snapshot()
            except ValueError:
                current = None
            if current is not None:
                off = {name: round(current[name] - q, 4) for name, q in final.items()
                       if abs(current[name] - q) > self.endpoint_tolerance}
                if not off:
                    return
            if time.monotonic() >= deadline:
                detail = off if current is not None else 'no current joint state'
                compliant = self.applied_impedance is not None and any(self.applied_impedance[0])
                raise RuntimeError(f'Driver reported success but joints stopped short of the '
                                   f'endpoint (rad): {detail}. '
                                   + ('In joint impedance the arm yields: raise impedance.stiffness '
                                      'or motion.endpoint_tolerance.' if compliant else
                                      'The stream may have timed out (STREAM TIMEOUT in the driver '
                                      'log). Send the target again, slower '
                                      '(motion.linear_velocity_limit, motion.minimum_time).'))

    def cancel(self):
        if self.goal_handle is not None and self.goal_handle.accepted:
            try:
                self.driver.wait(self.goal_handle.cancel_goal_async(), 5.0)
            except Exception as error:
                self.node.get_logger().error(f'trajectory cancel failed: {error}')
        self.goal_handle = None
