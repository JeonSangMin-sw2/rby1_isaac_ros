"""Planning client shared by the package's nodes. Contains no execution code.

`check_plan` and `benchmark` import only this, so neither can move a robot even
by mistake; `target_executor` adds the execution half on top.
"""

import math
import os
import time

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, PositionConstraint
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from rby1_cumotion.model import forward_kinematics, load_model, resolve_bundle

PIPELINES = ('isaac_ros_cumotion', 'ompl')
# One of each is the healthy shape. A second copy of any of them means an
# earlier launch was not reaped, and its controller_manager competes for the
# same names -- plans then succeed while their results never reach the client.
SINGLETON_NODES = ('move_group', 'cumotion_planner_node', 'ros2_control_node')


class PostureMismatch(ValueError):
    """The robot is not at the bundle posture. Waiting or retrying cannot fix it."""


def check_locked(metadata, positions, tolerance):
    unwatched = set(metadata.get('unwatched_joints', ()))  # the head: model.head_envelope
    for name, expected in metadata['locked_joints'].items():
        if name in unwatched:
            continue
        if name not in positions or not math.isfinite(positions[name]):
            raise ValueError(f'Missing/nonfinite locked joint: {name}')
        if abs(positions[name] - expected) > tolerance:
            raise PostureMismatch(
                f'Joint {name} is at {positions[name]:.4f} but the planner has it locked at '
                f'{expected:.4f}. Joints outside the planning group are fixed at the posture '
                'the cuMotion launch read when it started; the robot has moved since. '
                'Restart the launch (cumotion.launch.py or demo.launch.py) so it reads the '
                'posture again.')


def validate_trajectory(trajectory, metadata, current, tolerance=0.03):
    names = list(trajectory.joint_names)
    if len(names) != len(metadata['active_joints']) or set(names) != set(metadata['active_joints']):
        raise ValueError('Planner returned joints outside the selected group, or missing/duplicate joints')
    if len(trajectory.points) < 2:
        raise ValueError('Planner returned an empty or single-point trajectory')
    previous = -1.0
    for point in trajectory.points:
        if len(point.positions) != len(names) or not all(map(math.isfinite, point.positions)):
            raise ValueError('Malformed trajectory positions')
        for field in (point.velocities, point.accelerations, point.effort):
            if field and (len(field) != len(names) or not all(map(math.isfinite, field))):
                raise ValueError('Malformed trajectory derivatives/effort')
        stamp = point.time_from_start
        t = stamp.sec + stamp.nanosec * 1e-9
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 10**9 or t <= previous:
            raise ValueError('Trajectory timestamps must be nonnegative and strictly increasing')
        previous = t
        for name, q in zip(names, point.positions):
            limits = metadata['joint_limits'][name]
            if not limits['lower'] <= q <= limits['upper']:
                raise ValueError(f'Trajectory exceeds joint limits: {name}')
        for name, velocity in zip(names, point.velocities):
            if abs(velocity) > metadata['joint_limits'][name]['velocity'] + 1e-6:
                raise ValueError(f'Trajectory exceeds velocity limit: {name}')
    if any(abs(current[n] - q) > tolerance for n, q in zip(names, trajectory.points[0].positions)):
        raise ValueError('Robot moved away from the planned start state; replan before execution')


def quaternion(rotation):
    # Eigenvector form also handles rotations close to 180 degrees.
    r = rotation
    k = np.array([
        [r[0, 0] - r[1, 1] - r[2, 2], r[1, 0] + r[0, 1], r[2, 0] + r[0, 2], r[2, 1] - r[1, 2]],
        [r[1, 0] + r[0, 1], r[1, 1] - r[0, 0] - r[2, 2], r[2, 1] + r[1, 2], r[0, 2] - r[2, 0]],
        [r[2, 0] + r[0, 2], r[2, 1] + r[1, 2], r[2, 2] - r[0, 0] - r[1, 1], r[1, 0] - r[0, 1]],
        [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], r.trace()],
    ]) / 3.0
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


def check_transform(matrix, tolerance=1e-3):
    """A 4x4 rigid transform, or ValueError saying what is wrong with it.

    Targets arrive from outside as 16 numbers, so a transposed or unnormalised
    matrix is the likely mistake; it must not become a plausible-looking pose.
    """
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape == (16,):
        matrix = matrix.reshape(4, 4)
    if matrix.shape != (4, 4):
        raise ValueError(f'target must be 4x4 (or 16 values, row-major), got shape {matrix.shape}')
    if not np.isfinite(matrix).all():
        raise ValueError('target contains non-finite values')
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=tolerance):
        raise ValueError(f'target bottom row must be [0 0 0 1], got {matrix[3].tolist()} '
                         '-- is the matrix transposed? Values are read row-major.')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance):
        raise ValueError('target rotation is not orthonormal')
    if np.linalg.det(rotation) < 0:
        raise ValueError('target rotation is a reflection (determinant -1)')
    return matrix


class PlanningFailed(RuntimeError):
    """MoveIt answered, and the answer was no. `code` is a MoveItErrorCodes value."""

    def __init__(self, status, code):
        super().__init__(planning_failure(status, code))
        self.code = code


def warmup_offsets(offset):
    """`offset`, then the same length along the other base axes both ways.

    An obstacle can block any one direction; warmup only needs some plan.
    """
    offset = [float(v) for v in offset]
    length = float(np.linalg.norm(offset)) or 0.03
    candidates = [offset, [-v for v in offset]]
    for axis in (2, 0, 1):
        for sign in (1.0, -1.0):
            step = [0.0, 0.0, 0.0]
            step[axis] = sign * length
            candidates.append(step)
    unique = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    return unique


def planning_failure(status, code):
    """A MoveIt planning failure in words, with where to look next."""
    names = {getattr(MoveItErrorCodes, name): name for name in dir(MoveItErrorCodes)
             if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)}
    text = f'planning failed: {names.get(code, "UNKNOWN")} (MoveIt error {code}, action status {status})'
    if code == MoveItErrorCodes.PLANNING_FAILED:
        text += (' -- no collision-free motion to that pose: it may be out of reach, or an '
                 'obstacle may be in the way (scene list). The cumotion_planner log gives the '
                 'reason, e.g. IK_FAIL.')
    elif code == MoveItErrorCodes.INVALID_MOTION_PLAN:
        text += (' -- cuMotion found a path but MoveIt, checking it against its own robot model, '
                 'finds it in collision. MoveIt\'s model has links cuMotion\'s does not (the gripper beyond '
                 'the tool frame, robot.body_ends_at_tool): attach them as modules so cuMotion plans around '
                 'obstacles with them (rby1_moveit_objects config:=gripper.yaml), or name what may touch '
                 'in free_objects.')
    elif code == MoveItErrorCodes.TIMED_OUT:
        text += (' -- the MoveIt plugin waits only 5 s; see the cumotion_planner log. Send the '
                 'target again; if it keeps timing out, restart the cuMotion launch.')
    return text


def running_nodes(names=SINGLETON_NODES, proc='/proc'):
    """Command lines of the running processes that match each node name.

    Read from /proc rather than by shelling out to pgrep, whose own command line
    contains the pattern and so counts itself.
    """
    found = {name: [] for name in names}
    for entry in os.listdir(proc):
        if not entry.isdigit():
            continue
        try:
            with open(f'{proc}/{entry}/cmdline', 'rb') as stream:
                command = stream.read().decode('utf-8', 'replace').replace('\0', ' ').strip()
        except OSError:  # the process exited while we were looking
            continue
        if not command:
            continue
        # Match the executable only: a launch file naming the node in an
        # argument, or a shell whose script mentions it, is not that node.
        argv = command.split()
        executables = [argv[0]]
        # A ROS Python node runs as `python3 /path/to/node`, so the interpreter
        # hides the real name one argument along.
        if len(argv) > 1 and os.path.basename(argv[0]).startswith('python'):
            executables.append(argv[1])
        basenames = {os.path.basename(path) for path in executables}
        for name in names:
            if name in basenames:
                found[name].append(f'pid {entry}: {command[:120]}')
    return found


class PlanningClient(Node):
    """Loads a bundle, tracks joint state, and asks MoveIt for plan-only goals."""

    PARAMETERS = {
        'model_directory': '', 'group': '', 'pipeline': 'isaac_ros_cumotion',
        'offset_xyz': [0.03, 0.0, 0.0], 'velocity_scaling': 0.1,
        'acceleration_scaling': 0.1, 'planning_time': 30.0,
        'server_timeout': 120.0, 'state_max_age': 2.0,
        'locked_joint_tolerance': 0.01, 'joint_states_topic': '/joint_states',
        'warmup': True, 'warmup_attempts': 6, 'move_action': '/move_action',
        'controller_manager': '/controller_manager',
    }

    def __init__(self, name, extra=None):
        super().__init__(name)
        for key, default in {**self.PARAMETERS, **(extra or {})}.items():
            self.declare_parameter(key, default)
        # Empty model_directory: the runtime bundle the launch's prepare stage made.
        self.metadata, self.robot = load_model(resolve_bundle(self.param('model_directory')),
                                               self.param('group') or None)
        self.state = {}
        self.subscription = self.create_subscription(
            JointState, self.param('joint_states_topic'), self.on_state, qos_profile_sensor_data)
        self.planner = ActionClient(self, MoveGroup, self.param('move_action'))
        self.active_goal = None

    def param(self, name):
        return self.get_parameter(name).value

    def on_state(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for name, value in zip(msg.name, msg.position):
            self.state[name] = (value, stamp, time.monotonic())

    def snapshot(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        positions = {}
        for name in self.metadata['default_positions']:
            if name not in self.state:
                raise ValueError(f'Waiting for joint state: {name}')
            position, stamp, received = self.state[name]
            if (not math.isfinite(position) or abs(now - stamp) > self.param('state_max_age')
                    or time.monotonic() - received > self.param('state_max_age')):
                raise ValueError(f'Stale/nonfinite joint state: {name}')
            positions[name] = position
        check_locked(self.metadata, positions, self.param('locked_joint_tolerance'))
        return positions

    def wait(self, future, timeout, monitor=False):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= end:
                raise TimeoutError('ROS action timed out')
            rclpy.spin_once(self, timeout_sec=0.05)
            if monitor:
                self.snapshot()
        if not future.done():
            raise RuntimeError('ROS shut down while waiting for action')
        return future.result()

    def run_action(self, client, goal, timeout):
        self.active_goal = self.wait(client.send_goal_async(goal), 10.0)
        if not self.active_goal.accepted:
            self.active_goal = None
            raise RuntimeError('Action goal was rejected')
        result = self.wait(self.active_goal.get_result_async(), timeout, monitor=True)
        self.active_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PlanningFailed(result.status, result.result.error_code.val)
        return result.result

    def validate_parameters(self):
        offset = self.param('offset_xyz')
        if len(offset) != 3 or not all(map(math.isfinite, offset)) or np.linalg.norm(offset) > 0.1:
            raise ValueError('offset_xyz must contain 3 finite metres, with norm <= 0.1')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            if not 0.0 < self.param(name) <= 1.0:
                raise ValueError(f'{name} must be in (0, 1]')
        for name in ('planning_time', 'server_timeout', 'state_max_age', 'locked_joint_tolerance'):
            if not math.isfinite(self.param(name)) or self.param(name) <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.param('pipeline') not in PIPELINES:
            raise ValueError(f'pipeline must be one of {PIPELINES}')
        return offset

    def await_planner(self):
        if not self.planner.wait_for_server(timeout_sec=self.param('server_timeout')):
            raise TimeoutError('MoveIt move_action server is unavailable')

    def active_controllers(self, attempts=3, timeout=15.0):
        """Names of the controllers the controller_manager reports as active.

        Retried: the controller_manager occasionally logs `failed to send
        response to /controller_manager/list_controllers (timeout)` and the
        reply never arrives. That is a transient DDS failure, and letting it
        abort a benchmark would make the environment check its own flake source.
        """
        service = f'{self.param("controller_manager")}/list_controllers'
        client = self.create_client(ListControllers, service)
        try:
            if not client.wait_for_service(timeout_sec=self.param('server_timeout')):
                raise TimeoutError(f'{service} is unavailable')
            for attempt in range(1, attempts + 1):
                try:
                    response = self.wait(client.call_async(ListControllers.Request()), timeout)
                    return sorted(c.name for c in response.controller if c.state == 'active')
                except TimeoutError:
                    if attempt == attempts:
                        raise TimeoutError(f'{service} did not answer in {attempts} attempts')
                    self.get_logger().warn(f'{service} attempt {attempt} timed out; retrying')
        finally:
            self.destroy_client(client)

    def fresh_snapshot(self, timeout=10.0):
        """Spin until a complete, current joint state is available."""
        end = time.monotonic() + timeout
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self.snapshot()
            except PostureMismatch:
                raise  # a complete, current state that is simply elsewhere
            except ValueError:
                if time.monotonic() >= end:
                    raise

    def tool_pose(self, positions=None):
        """The tool frame in the base frame, as a 4x4 homogeneous transform."""
        positions = positions if positions is not None else self.fresh_snapshot()
        return forward_kinematics(self.robot, positions, self.metadata['tool_frame'])

    def plan(self, offset):
        """Plan to the current tool pose moved by `offset` (base frame, metres)."""
        target = self.tool_pose().copy()
        target[:3, 3] += np.asarray(offset, dtype=float)
        return self.plan_to(target)

    def plan_to(self, target):
        """Plan to a 4x4 tool pose in the base frame; returns the result and the request."""
        target = check_transform(target)
        positions = self.fresh_snapshot()
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = target[:3, 3].tolist()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
            quaternion(target[:3, :3]).tolist()
        target = pose
        position = PositionConstraint()
        position.header.frame_id = self.metadata['base_frame']
        position.link_name = self.metadata['tool_frame']
        position.weight = 1.0
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
        position.constraint_region.primitive_poses = [target]
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.metadata['base_frame']
        orientation.link_name = self.metadata['tool_frame']
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.metadata['group']
        request.pipeline_id = self.param('pipeline')
        request.allowed_planning_time = self.param('planning_time')
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = self.param('velocity_scaling')
        request.max_acceleration_scaling_factor = self.param('acceleration_scaling')
        request.start_state.joint_state.name = list(positions)
        request.start_state.joint_state.position = list(positions.values())
        request.start_state.joint_state.velocity = [0.0] * len(positions)
        request.start_state.is_diff = False
        request.goal_constraints = [Constraints(position_constraints=[position], orientation_constraints=[orientation])]
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        result = self.run_action(self.planner, goal, self.param('planning_time') + 30.0)
        return result, request

    def measure(self, offset=None, target=None):
        """One validated plan, to `target` (4x4) or else the current pose + `offset`."""
        begin = time.monotonic()
        result, request = self.plan_to(target) if target is not None else self.plan(offset)
        wall = time.monotonic() - begin
        trajectory = result.planned_trajectory.joint_trajectory
        validate_trajectory(trajectory, self.metadata, self.snapshot())
        stamp = trajectory.points[-1].time_from_start
        return {'planner_time': result.planning_time, 'wall_time': wall,
                'points': len(trajectory.points),
                'duration': stamp.sec + stamp.nanosec * 1e-9,
                'pipeline': request.pipeline_id, 'group': request.group_name,
                'trajectory': result.planned_trajectory}

    def warm(self):
        """Absorb cuMotion's first plan, which the MoveIt plugin cannot wait for.

        That first call captures CUDA graphs and takes about ten seconds for a
        13-joint group, while the plugin waits a hardcoded five and then
        abandons the goal. cuMotion keeps working, so the next request is
        refused with 'Planner is busy' until it finishes. Retrying here means a
        caller's first real query is never the one that pays for this.

        A plain 'no path' answer means the planner is up and an obstacle blocks
        that direction, so the next direction is tried at once.
        """
        directions = warmup_offsets(self.param('offset_xyz'))
        last = None
        for attempt in range(1, int(self.param('warmup_attempts')) + 1):
            started = time.monotonic()
            try:
                for index, offset in enumerate(directions):
                    try:
                        self.plan(offset)
                        break
                    except PlanningFailed as error:
                        if error.code != MoveItErrorCodes.PLANNING_FAILED or index == len(directions) - 1:
                            raise
                self.get_logger().info(f'warmup done in {time.monotonic() - started:.1f}s '
                                       f'after {attempt} attempt(s)')
                return
            except PostureMismatch:
                raise  # the planner is fine; the robot is elsewhere, and stays so
            except Exception as error:
                last = error
                self.get_logger().info(f'warmup attempt {attempt} not ready: {error}')
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(f'Planner never became ready. Last answer: {last}. Usually an obstacle '
                           'touches the arm (ros2 run rby1_moveit_objects scene list / clear), or '
                           "cuMotion is still starting (wait for 'cuMotion is ready for planning "
                           "queries' and start again).")

    def await_joint_states(self, timeout=10.0):
        """Fail once, clearly, when no robot state arrives -- not per warmup attempt.

        Joint states come from the host driver (relayed to /joint_states by
        the cuMotion launch), or from ros2_control with hardware:=mock. When the driver
        stops, the planner stack stays up but nothing publishes, and every plan
        would wait and retry for over a minute saying only 'Waiting for joint state'.
        """
        try:
            self.fresh_snapshot(timeout)
        except PostureMismatch:
            raise
        except ValueError as error:
            topic = self.param('joint_states_topic')
            raise RuntimeError(
                f'No current robot joint states on {topic} after {timeout:.0f} s ({error}). '
                'Is the RB-Y1 driver running on the host, on the same ROS_DOMAIN_ID? '
                'Start it (ros2 launch rby1_driver rby1_ros2_driver.launch.py); the cuMotion '
                'launch relays its /rby1/joint_states and picks it up again.') from None

    def prepare(self):
        """Validate parameters, wait for MoveIt and joint states, warm the planner.
        Returns the offset."""
        offset = self.validate_parameters()
        self.await_planner()
        self.await_joint_states()
        if self.param('warmup'):
            self.warm()
        return offset

    def cancel_active_goal(self):
        if not (self.active_goal and self.active_goal.accepted):
            return
        try:
            response = self.wait(self.active_goal.cancel_goal_async(), 5.0)
            if not response.goals_canceling:
                self.get_logger().error('Action cancellation was not acknowledged')
        except Exception as error:
            self.get_logger().error(f'Action cancellation failed: {error}')


def run_node(factory):
    """Shared entry point: build the node, run it, cancel any goal on failure."""
    rclpy.init()
    node = None
    exit_code = 0
    try:
        node = factory()
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        exit_code = 1
        if node:
            node.get_logger().error(str(error) or 'Interrupted')
            node.cancel_active_goal()
        else:
            print(f'{factory.__name__} failed to start: {error}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
