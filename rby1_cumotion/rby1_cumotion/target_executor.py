"""The one node that moves the robot: move to 4x4 targets from a topic.

cumotion.launch.py starts it after its prepare stage has checked the robot,
powered it, bent a straight planning arm to the ready pose and built the runtime
bundle. On start it checks the robot once more through the host RB-Y1 driver --
emergency stop, control manager faults, the robot kind against the bundle --
turns power and servos on (robot.enable_robot) and warms the planner. Then it
waits.

Each message on `target_topic` is a std_msgs/Float64MultiArray of 16 values: a
row-major homogeneous transform of the group's tool frame in the bundle's base
frame. For each one: plan with cuMotion, time the path (motion.* in
config/cumotion.yaml), and run it through the driver's follow_joint_trajectory.
Progress goes out on `status_topic` as plain strings: READY, PLANNING,
EXECUTING, DONE, REPLACED, or FAILED: <reason>.

How long a move takes: `duration` > 0 fixes it. With 0 it is the longest of
`minimum_time` and what the tool's linear and angular speed limits and the
joints' velocity limits allow along that path. All motion.* parameters can be
changed while it runs (ros2 param set); each change is range-checked.

A target that arrives while another is running replaces it (REPLACED): with
avoid.enabled the arm is replanned toward it from where it is and how fast it
moves, without a stop; otherwise it runs once the current move ends. Only the
latest one counts -- a burst of messages does not queue up into a string of
motions nobody is watching.

With avoid.enabled, obstacles that appear during a move are handled while it
runs (avoidance.py): the next avoid.horizon seconds are checked. A standing
obstacle ahead makes cuRobo plan again from the arm's position and speed
avoid.lead seconds later; the new trajectory replaces the running one in the
driver without a stop. A moving one makes the arm brake on its path short of it
(WAITING) and carry on once the rest of the path is clear (RESUMING) -- or, if
the obstacle stopped on the path, plan around it from there. Blocked for
avoid.wait_before_replan while stopped, it plans a new path from where it stands,
at most avoid.max_replans times per target. It fails when the target has no
collision-free pose, when the obstacle is too close to plan around or stop short
of in time, when avoiding lasts avoid.give_up_after seconds, or when
avoid.max_replans new paths did not get through.

Moves the robot.
"""

import collections
import time

import numpy as np
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rcl_interfaces.srv import GetParameters
import rclpy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rby1_msgs.msg import RobotState
from rby1_cumotion import planner_params, settings
from rby1_cumotion.attached import allow_touching, attached_spheres, flatten, free_names
from rby1_cumotion.avoidance import (Avoider, clear_of, HOPELESS, MotionTracker, objects_from_scene,
                                     swept)
from rby1_cumotion.driver import check_driver_model, robot_problem
from rby1_cumotion.execution import (ahead, back_off, DriverExecutor, impedance_request, motion_duration,
                                     resume_from, retime, seconds, slow_to_stop, splice, stamp,
                                     state_at, times_of, ToolChain)
from rby1_cumotion.model import ATTACHED_LINK
from rby1_cumotion.planning import check_transform, PlanningClient, run_node
from rby1_cumotion.tracking import Servo, TargetMotion, Tracker

MOTION = tuple(settings.SECTIONS['motion'])
# Tracking mode carries a target on along its velocity for this long while the next one is due.
CARRY_ON = 0.1
# Tracking mode restarts from the measured arm when a joint is off its command by more than
# resync_tolerance plus what it travels in RESYNC_SLACK s, for RESYNC_TICKS ticks in a row.
RESYNC_SLACK = 0.1
RESYNC_TICKS = 5
IMPEDANCE = tuple(f'impedance.{name}' for name in settings.SECTIONS['impedance'])
# Waypoint spacing of cuRobo's replanned path (Avoider's interpolation_dt).
PLAN_DT = 0.025
# Waiting for a moving obstacle: how far back along the path to try (s of the
# path's own timing), how long the waiting pose must stay clear, and how early or
# late the obstacle may be against its measured velocity.
BACK_OFFS = (0.25, 0.5, 1.0, 1.5)
WAIT_WINDOW = 2.0
TIME_MARGIN = 0.3


def hopeless(status):
    """Why a replan that cannot succeed stopped the arm."""
    if status.startswith('INVALID_START_STATE'):
        return (f'obstacle already touches the arm ({status}); stopped. Move it off the arm '
                '(ros2 run rby1_moveit_scene scene list) and send the target again')
    return (f'no way around the obstacle to the target ({status}); stopped. Move the obstacle '
            '(ros2 run rby1_moveit_scene scene list) or send another target')


class TargetExecutor(PlanningClient):
    def __init__(self):
        # Name and topics shared with rby1_moveit_executor: whichever planner runs,
        # targets and `ros2 param set /rby1_target_executor ...` go to the same place.
        super().__init__('rby1_target_executor', extra={
            'config': '',
            'target_topic': '/rby1/target_pose',
            'status_topic': '/rby1/target_status',
            'attach_action': '/planner_attach_object',  # the cuMotion planner's UpdateLinkSpheres
        })
        # Defaults for the rest come from the settings file, like everything else.
        config = settings.load(self.param('config') or None)
        for name in ('driver_namespace', 'enable_robot'):
            self.declare_parameter(name, config['robot'][name])
        # [''] is an empty list of strings to ROS.
        self.declare_parameter('free_objects', list(config['robot']['free_objects']) or [''])
        for name in MOTION:
            self.declare_parameter(name, config['motion'][name])
        for name in settings.SECTIONS['impedance']:
            self.declare_parameter(f'impedance.{name}', config['impedance'][name])
        self.avoid = config['avoid']
        self.tracking_cfg = config['tracking']
        self.follower, self.tracking = None, False  # the tracking-mode MPC (self.tracker: moving obstacles)
        self.create_service(SetBool, '~/set_tracking', self.on_set_tracking)
        # Read by the examples: obstacles that move need a planner that watches the way.
        self.declare_parameter('watches_while_moving', bool(self.avoid['enabled']),
                               ParameterDescriptor(read_only=True))
        self.avoider = None
        if self.avoid['enabled']:
            path = self.param('config') or str(settings.default_path())
            planner = planner_params.parse(lambda name: planner_params.FROM_CONFIG,
                                           planner_params.load_config(path))
            self.get_logger().info('loading cuRobo for obstacle avoidance while moving (avoid.enabled)')
            self.avoider = Avoider(self.metadata, self.robot, planner, ground=planner['add_ground_plane'],
                                   seeds=self.avoid['replan_seeds'], steps=self.avoid['replan_steps'],
                                   iters=self.avoid['replan_iters'])
            self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
            self.scene_future, self.scene_read = None, 0.0
            self.scene = ([], {}, 0.0)
            self.tracker = MotionTracker()
        self.add_on_set_parameters_callback(self.check_motion_change)
        self.trajectory_runner = DriverExecutor(self, self.param('driver_namespace'))
        self.trajectory_runner.impedance = lambda: impedance_request(
            {name: self.param(f'impedance.{name}') for name in settings.SECTIONS['impedance']},
            self.metadata['active_joints'], self.metadata['default_positions'])
        self.trajectory_runner.stream_is_on = lambda: bool(
            self.robot_state is not None and self.robot_state.robot_stream_state)
        self.driver = self.trajectory_runner.driver
        self.chain = ToolChain(self.robot, self.metadata['tool_frame'])
        self.robot_state = None
        self.create_subscription(RobotState, self.driver.name('robot_state'), self.on_robot_state, 10)
        self.status = self.create_publisher(String, self.param('status_topic'), 10)
        self.create_subscription(Float64MultiArray, self.param('target_topic'), self.on_target, 10)
        self.pending, self.pending_at = None, 0.0
        self.busy = False
        # Modules attached to robot links in MoveIt's scene, mirrored to cuMotion (attached.py).
        self.attached_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.attached_future, self.attached_read, self.attached_key = None, 0.0, None
        # MoveIt checks every path against its own model (the URDF, gripper included):
        # what is free for cuMotion must be allowed to touch there too.
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.collision_matrix, self.freed = None, []
        self.attach_client = None

    def check_motion_change(self, parameters):
        for parameter in parameters:
            if parameter.name in MOTION or parameter.name in IMPEDANCE:
                section, name = (('motion', parameter.name) if parameter.name in MOTION
                                 else parameter.name.split('.', 1))
                try:
                    settings.coerce(section, name, parameter.value, 'ros2 param set')
                except ValueError as error:
                    return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def on_robot_state(self, msg):
        self.robot_state = msg

    def report(self, text):
        self.status.publish(String(data=text))
        # One call site per severity: rclpy raises 'Logger severity cannot be
        # changed between calls' when a single line logs at two levels.
        if text.startswith('FAILED'):
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def check_robot(self, timeout=15.0):
        """Driver reachable, robot kind right, no e-stop or major fault; then power up."""
        response = self.driver.call('rby1_ros2_driver/get_parameters', GetParameters,
                                    GetParameters.Request(names=['model', 'robot_ip']))
        driver_model, robot_ip = (value.string_value for value in response.values)
        check_driver_model(driver_model, self.metadata['model'])
        deadline = time.monotonic() + timeout
        while self.robot_state is None:
            if time.monotonic() > deadline:
                raise TimeoutError(f'No {self.driver.name("robot_state")} from the driver')
            rclpy.spin_once(self, timeout_sec=0.1)
        problem = robot_problem(self.robot_state)
        if problem:
            raise RuntimeError(problem)
        self.get_logger().info(f'robot: {self.metadata["model"]} at {robot_ip}')
        if self.param('enable_robot'):
            for service in ('robot_power', 'robot_servo'):
                self.driver.switch(service, True)
            self.get_logger().info('power and servos on')

    def move_duration(self, trajectory):
        """Seconds for this path, and what decided it."""
        fixed = self.param('duration')
        if fixed > 0:
            return fixed, 'duration'
        seconds, needed = motion_duration(
            trajectory, self.chain, self.snapshot(), self.metadata['joint_limits'],
            self.param('linear_velocity_limit'), self.param('angular_velocity_limit'),
            self.param('minimum_time'))
        return seconds, max(needed, key=needed.get)

    def refresh_scene(self, wait=False):
        """Read the planning scene now and then; hand the obstacles (swept ahead) to cuRobo."""
        now = time.monotonic()
        if self.scene_future is None and (wait or now - self.scene_read >= self.avoid['scene_period']):
            if not self.scene_client.service_is_ready():
                if not wait:
                    return
                if not self.scene_client.wait_for_service(timeout_sec=10.0):
                    raise TimeoutError('/get_planning_scene unavailable -- is move_group running?')
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            self.scene_future = self.scene_client.call_async(request)
            self.scene_read = now
        if self.scene_future is None:
            return
        if wait:
            self.wait(self.scene_future, 10.0)
        if not self.scene_future.done():
            return
        response, self.scene_future = self.scene_future.result(), None
        objects = objects_from_scene(response.scene.world.collision_objects)
        moving = self.tracker.update(objects, now)
        self.scene = (objects, moving, now)
        # Standing obstacles go into cuRobo's world for the check; moving ones are
        # checked at each waypoint's own time (avoidance.moving_collision).
        self.avoider.update([obj for obj in objects if obj['name'] not in moving],
                            swept(objects, moving, self.avoid['sweep_time']))

    def sync_attached(self, wait=False):
        """Hand cuMotion the modules attached to the hand, when they changed (read every 1 s)."""
        now = time.monotonic()
        if self.attached_future is None and (wait or now - self.attached_read >= 1.0):
            if not self.attached_scene.service_is_ready():
                return
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            self.attached_future = self.attached_scene.call_async(request)
            self.attached_read = now
        if self.attached_future is None:
            return
        if wait:
            self.wait(self.attached_future, 10.0)
        if not self.attached_future.done():
            return
        response, self.attached_future = self.attached_future.result(), None
        objects = list(response.scene.robot_state.attached_collision_objects)
        free = [name for name in self.param('free_objects') if name]
        key = [free, *((a.link_name, str(a.object)) for a in objects)]
        if key == self.attached_key:
            return
        try:
            spheres, notes = attached_spheres(objects, self.metadata, self.robot, free=free)
        except ValueError as error:
            self.get_logger().error(f'attached modules: {error}')
            self.attached_key = key
            return
        self.allow_in_moveit(free_names(objects, self.robot, free))
        self.send_link_spheres(spheres)
        if self.avoider is not None:
            self.avoider.attach(spheres, ATTACHED_LINK)
        if self.follower is not None:
            self.follower.attach(spheres, ATTACHED_LINK)
        self.attached_key = key
        for name, note in notes.items():
            self.get_logger().info(f'attached module {name}: {note}')
        if not objects:
            self.get_logger().info('no attached modules')
        if not spheres and self.metadata.get('body_ends_at_tool'):
            tool = self.metadata['tool_frame']
            self.get_logger().warn(
                f'nothing on {tool} is in cuMotion\'s model: it plans as if the hand ended at {tool} (the arm\'s '
                'body stops there, robot.body_ends_at_tool). Attach what is mounted on it -- '
                'ros2 launch rby1_moveit_objects objects.launch.py config:=gripper.yaml -- or set '
                'robot.body_ends_at_tool false')

    def allow_in_moveit(self, names):
        """MoveIt's allowed-collision matrix: `names` (attached objects, robot links) may touch anything."""
        if names == self.freed:
            return
        if self.collision_matrix is None:  # as it was before anything was freed
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            future = self.attached_scene.call_async(request)
            self.wait(future, 10.0)
            self.collision_matrix = future.result().scene.allowed_collision_matrix
        scene = PlanningScene(is_diff=True)
        scene.allowed_collision_matrix = allow_touching(self.collision_matrix, names)
        future = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene))
        self.wait(future, 10.0)
        if not future.result().success:
            raise RuntimeError('move_group refused the allowed-collision update for free_objects')
        self.freed = names
        self.get_logger().info(f'free to touch anything (cuMotion and MoveIt): {names}' if names
                               else 'nothing is free to touch')

    def send_link_spheres(self, spheres):
        """The planner's attached-object spheres: cleared, then set (it overwrites from slot 0)."""
        from isaac_ros_cumotion_interfaces.action import UpdateLinkSpheres
        from rclpy.action import ActionClient
        if self.attach_client is None:
            self.attach_client = ActionClient(self, UpdateLinkSpheres, self.param('attach_action'))
        if not self.attach_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'{self.param("attach_action")} unavailable -- is the cuMotion planner running?')
        for attach in ([False, True] if spheres else [False]):
            goal = UpdateLinkSpheres.Goal(attach_object=attach, object_link_name=ATTACHED_LINK,
                                          flattened_sphere_arr=flatten(spheres) if attach else [])
            handle = self.wait(self.attach_client.send_goal_async(goal), 5.0)
            if not handle.accepted:
                raise RuntimeError('the cuMotion planner refused the attached-module spheres')
            self.wait(handle.get_result_async(), 5.0)

    # -- tracking mode (tracking.py) -----------------------------------------------------------

    def build_tracker(self):
        if self.follower is not None:
            return
        started = time.monotonic()
        cfg = self.tracking_cfg
        ground = self.avoider.ground if self.avoider is not None else False
        if cfg['method'] == 'ik':
            self.follower = Servo(self.metadata, self.robot, cfg['rate'], cfg['max_speed'], cfg['max_acceleration'],
                                  ground=ground)
            detail = f'IK servo, joints within {cfg["max_speed"]:g} rad/s and {cfg["max_acceleration"]:g} rad/s^2'
        else:
            self.follower = Tracker(self.metadata, self.robot, cfg['rate'], cfg['iterations'], ground=ground)
            detail = f'MPC, {cfg["iterations"]} iteration'
        current = self.fresh_snapshot()
        self.follower.warm_up([current[name] for name in self.follower.names])
        self.get_logger().info(f'tracking ready in {time.monotonic() - started:.1f}s ({cfg["rate"]:.0f} Hz, {detail})')

    def on_set_tracking(self, request, response):
        """std_srvs/SetBool: true -- targets move an MPC goal (tracking); false -- plan each one (p2p)."""
        if request.data == self.tracking:
            response.success, response.message = True, f'already {"tracking" if self.tracking else "p2p"}'
        elif request.data and self.busy:
            response.success = False
            response.message = 'a move is in progress; switch once it is DONE or FAILED'
        else:
            self.tracking = request.data
            response.success = True
            response.message = ('tracking: targets on {} move the goal, followed at {:.0f} Hz'.format(
                self.param('target_topic'), self.tracking_cfg['rate']) if request.data
                else 'p2p: each target is planned and executed')
        return response

    def track(self):
        """Follow the targets with the MPC until tracking is switched off."""
        from rby1_msgs.action import StreamJoint
        from rclpy.action import ActionClient
        try:
            self.build_tracker()
        except Exception as error:
            self.tracking = False
            raise RuntimeError(f'tracking failed to start: {error}') from error
        follower, cfg = self.follower, self.tracking_cfg
        names = follower.names
        stream = ActionClient(self, StreamJoint, self.driver.name('stream_joint'))
        if not stream.wait_for_server(timeout_sec=5.0):
            self.tracking = False
            raise RuntimeError(f'{self.driver.name("stream_joint")} unavailable -- is the driver running?')
        measured = self.fresh_snapshot()
        follower.start([measured[name] for name in names])
        self.sync_attached(wait=True)
        owns = not (self.robot_state is not None and self.robot_state.robot_stream_state)
        self.trajectory_runner.apply_impedance()  # stream_joint follows the same switch
        self.driver.switch('stream_control', True)
        refused = {'count': 0}

        def answered(future):
            handle = future.result()
            refused['count'] = 0 if handle.accepted else refused['count'] + 1
        period = 1.0 / cfg['rate']
        parts = {part: [i for i, name in enumerate(names) if name.startswith(part + '_')]
                 for part in ('torso', 'right_arm', 'left_arm')}
        last_target, holding, tick = None, False, time.monotonic()
        note, astray = None, 0
        measured_at = tick
        sent = collections.deque()  # (time, positions): the measured arm follows command_delay later
        motion = TargetMotion(cfg['smoothing'], reset_after=cfg['stale_after'])
        stats = {'ticks': 0, 'step': 0.0, 'since': time.monotonic()}
        self.pending = None
        self.report(f'TRACKING at {cfg["rate"]:.0f} Hz: send targets on {self.param("target_topic")}')
        try:
            while self.tracking and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                if self.avoider is not None:
                    self.refresh_scene()
                    objects, moving, _ = self.scene
                    follower.world([obj for obj in objects if obj['name'] not in moving]
                                   + swept(objects, moving, self.avoid['sweep_time']))
                self.sync_attached()
                now = time.monotonic()
                if self.pending is not None:
                    # Timed when it arrived, not at this tick: ticks are 20 ms apart, targets
                    # from a 30 Hz camera 33 ms, and the speed is their difference over that time.
                    motion.update(self.pending, self.pending_at)
                    self.pending, last_target = None, self.pending_at
                    if holding:
                        holding = False
                        self.report('TRACKING: following again')
                elif last_target is not None and not holding and now - last_target > cfg['stale_after']:
                    holding = True  # the goal stays: the arm settles at the last target
                    follower.aim(motion.at())  # no longer moving: aim at it, not ahead of it
                    self.report(f'TRACKING: no target for {cfg["stale_after"]:.1f}s; holding at the last one')
                if last_target is not None and not holding:
                    # Aim where a moving target will be -- the follower and the robot trail it --
                    # every tick, from where the targets put it, how fast it moves and how old
                    # the last one is.
                    ahead = cfg['lead'] + follower.lag
                    follower.aim(motion.at(now, ahead, cfg['lead_max'], CARRY_ON),
                                 motion.heading(now, ahead, CARRY_ON))
                stepped = time.monotonic()
                positions, speeds = follower.step()
                stats['step'] += time.monotonic() - stepped
                if follower.note != note:  # the IK servo cannot follow: say why, and when it can again
                    note = follower.note
                    self.report(f'TRACKING: {note}' if note else 'TRACKING: following again')
                stats['ticks'] += 1
                if now - stats['since'] >= 2.0:
                    self.get_logger().debug(f'tracking: {stats["ticks"] / (now - stats["since"]):.1f} Hz, step '
                                            f'{stats["step"] / stats["ticks"] * 1000:.1f} ms')
                    stats.update(ticks=0, step=0.0, since=now)
                try:
                    current, measured_at = self.snapshot(), now
                except ValueError as error:  # a late joint state: check the next tick
                    if now - measured_at > 1.0:
                        raise RuntimeError(f'no current joint states for 1 s ({error})') from error
                    current = None
                sent.append((now, positions.copy(), float(np.abs(speeds).max())))
                while len(sent) > 1 and sent[1][0] <= now - cfg['command_delay']:
                    sent.popleft()
                then = sent[0][1]  # what was commanded command_delay ago
                lag, off = (0.0, None) if current is None else max(
                    (abs(current[name] - then[i]), name) for i, name in enumerate(names))
                # How late the arm follows is known to about RESYNC_SLACK s: a fast joint may be
                # that much of its way off without being stuck. And stuck lasts; one late joint
                # state does not.
                allowed = cfg['resync_tolerance'] + RESYNC_SLACK * max(entry[2] for entry in sent)
                astray = astray + 1 if lag > allowed else 0
                if astray >= RESYNC_TICKS:
                    self.get_logger().warn(f'{off} is {lag:.2f} rad from its command: carrying on from where '
                                           'the arm is')
                    follower.resync([current[name] for name in names])
                    sent.clear()
                    astray = 0
                    positions = np.array([current[name] for name in names])
                goal = StreamJoint.Goal()
                for part, index in parts.items():
                    if index:
                        command = getattr(goal.command, part)
                        command.position = [float(positions[i]) for i in index]
                        command.minimum_time = period
                stream.send_goal_async(goal).add_done_callback(answered)
                if refused['count'] > int(cfg['rate']):  # a second of refusals
                    raise RuntimeError('the driver refuses stream_joint (a trajectory or another command holds the '
                                       'arm, or the stream closed) -- see the driver log')
                tick += period
                while time.monotonic() < tick:
                    rclpy.spin_once(self, timeout_sec=max(0.0, tick - time.monotonic()))
                if time.monotonic() - tick > 5 * period:
                    tick = time.monotonic()  # fell behind: do not rush to catch up
        finally:
            self.tracking = False
            self.pending = None
            if owns:
                try:
                    self.driver.switch('stream_control', False)
                except Exception as error:
                    self.get_logger().warn(f'stream_control off: {error}')
                self.trajectory_runner.stream_closed = time.monotonic()
            stream.destroy()
        self.report('READY')

    def execute_avoiding(self, trajectory, target):
        """Run `trajectory`, getting past obstacles that come into its way.

        A standing obstacle ahead: plan a way around it and take it over without a
        stop (avoid_once). A moving one: brake on the path short of it and wait
        until it is out of the way (brake, wait_out).
        """
        runner = self.trajectory_runner
        names = list(trajectory.joint_names)
        self.refresh_scene(wait=True)
        self.waiting = None
        self.replans = 0
        try:
            runner.begin(trajectory)
            since, last_try = None, 0.0
            while not runner.done():
                rclpy.spin_once(self, timeout_sec=self.avoid['check_period'])
                self.refresh_scene()
                t_now = runner.now()
                if self.pending is not None:
                    target = self.retarget(names, t_now)
                    since = None
                    continue
                if self.waiting is not None:
                    self.wait_out(target, names, t_now)
                    continue
                rows = ahead(runner.sent, t_now, self.avoid['horizon'])
                hit, by_moving = self.collision(rows, runner.step * np.arange(1, len(rows) + 1), names)
                if hit is None:
                    since = None
                    continue
                t_hit = t_now + (hit + 1) * runner.step
                self.get_logger().debug(f'hit: row {hit} at +{(hit + 1) * runner.step:.2f}s of {len(rows)}, '
                                        f't_now={t_now:.2f} moving={by_moving}')
                if by_moving:
                    self.brake(names, t_now, t_hit)
                    since = None
                    continue
                now = time.monotonic()
                since = since or now
                if now - since > self.avoid['give_up_after']:
                    raise RuntimeError(f'obstacle still in the way after {self.avoid["give_up_after"]:.1f} s '
                                       'of avoiding; stopped. Send the target again once it is '
                                       'gone (the time is avoid.give_up_after)')
                if now - last_try < 0.1:
                    continue
                last_try = now
                self.avoid_once(target, names, t_now, t_hit)
            final = runner.sent
        except Exception:
            runner.stop()
            raise
        finally:
            self.waiting = None
        runner.finish(final, self.snapshot)

    def collision(self, rows, times, names):
        """(first of `rows` -- the arm `times` s from now -- that meets an obstacle, by a moving one?)."""
        objects, moving, read = self.scene
        # Moving obstacles are compared where they will be then, counted from the reading.
        return self.avoider.first_hit(rows, names, np.asarray(times) + (time.monotonic() - read),
                                      [obj for obj in objects if obj['name'] in moving], moving)

    def trajectory_collision(self, trajectory, names):
        rows = np.array([point.positions for point in trajectory.points], dtype=float)
        return self.collision(rows, times_of(trajectory), names)

    def plan_around(self, q, names):
        """cuRobo's planning world for a replan from `q`."""
        objects, moving, _ = self.scene
        # Plan clear of where moving obstacles are going -- except the stretch of
        # their way that covers where the arm is now: the arm leaves before they
        # get there, but the planner would read it as starting inside them.
        standing = [obj for obj in objects if obj['name'] not in moving]
        ahead_copies = [obj for obj in swept(objects, moving, self.avoid['sweep_time'])
                        if obj['name'] not in {o['name'] for o in standing}]
        self.avoider.plan_world(standing + clear_of(ahead_copies, self.avoider.spheres(q, names)))

    def avoid_once(self, target, names, t_now, t_hit):
        """Replan from avoid.lead ahead (never past the collision) and swap trajectories."""
        runner = self.trajectory_runner
        t_join = min(t_now + self.avoid['lead'], runner.motion_end())
        if t_join >= t_hit - runner.step:
            t_join = t_hit - runner.step
        if t_join < t_now + self.avoid['replan_time']:
            raise RuntimeError('obstacle too close to plan around in time; stopped. Send the '
                               'target again; slower moves leave more time to get around '
                               '(motion.linear_velocity_limit)')
        q, v = state_at(runner.sent, t_join)
        started = time.monotonic()
        self.plan_around(q, names)
        path, status = self.avoider.replan(q, v, target, names)
        planned_in = time.monotonic() - started
        if path is None:
            if status in HOPELESS:
                raise RuntimeError(hopeless(status))
            self.get_logger().warn(f'avoiding: replan failed ({status}); trying again')
            return
        t_now = runner.now()
        if t_now >= t_join - runner.step:
            self.get_logger().warn(f'avoiding: replan took {time.monotonic() - started:.2f} s, too late '
                                   'to take over; trying again')
            return
        new = splice(runner.sent, t_now, t_join, path, self.metadata['joint_limits'],
                     runner.step, floor=self.detour_time(path, names))
        runner.replace(new)
        _, v_at_join = state_at(new, t_join - t_now)
        planned = [float(np.linalg.norm(state_at(new, t)[1]))
                   for t in np.arange(t_join - t_now, t_join - t_now + 1.0, runner.step)]
        self.report(f'AVOIDING replanned in {time.monotonic() - started:.2f}s (cuRobo {planned_in:.2f}s), takes over '
                    f'{t_join - t_now:.2f}s ahead; joint speed there {np.linalg.norm(v_at_join):.3f} rad/s, '
                    f'planned lowest in the next 1 s {min(planned):.3f} rad/s')

    def brake(self, names, t_now, t_hit):
        """A moving obstacle ahead: stop on the path short of it and wait there.

        Tries the gentlest braking (avoid.stop_time) first, then harder ones. If
        wherever it would stop is in the obstacle's way too, it backs along the
        path it came -- known to be clear of everything standing.
        """
        runner = self.trajectory_runner
        current, end = runner.sent, runner.motion_end()
        room = t_hit - runner.step - t_now  # trajectory time left before the collision
        stops = [t for t in (self.avoid['stop_time'], self.avoid['stop_time'] / 2, 2 * runner.step)
                 if t / 2 <= room]
        if not stops:
            raise RuntimeError('moving obstacle too close to stop short of it; stopped. Send the '
                               'target again once it has passed')
        hold = self.wait_limit() + 2.0  # keeps the driver's stream going while waiting
        options = []  # gentlest braking first; for each, stopping there, then backing off
        for stop_time in stops:
            stop, s_stop = slow_to_stop(current, t_now, stop_time, hold, runner.step)
            options.append((f'braking over {stop_time:.2f}s', stop, s_stop))
            braking, _ = slow_to_stop(current, t_now, stop_time, 0.0, runner.step)
            for back in BACK_OFFS:
                s_rest = max(s_stop - back, times_of(current)[0])
                options.append((f'braking over {stop_time:.2f}s and backing {s_stop - s_rest:.2f}s along the path',
                                back_off(current, s_stop, s_rest, hold, runner.step, start=braking,
                                         limits=self.metadata['joint_limits']), s_rest))
        for text, trajectory, s_rest in options:
            if self.clear_while_waiting(trajectory, hold, names):
                runner.replace(trajectory)
                self.waiting = {'path': current, 's_rest': s_rest, 'end': end, 'clear': 0,
                                'rest_at': seconds(trajectory.points[-1].time_from_start) - hold,
                                'since': time.monotonic(), 'blocked_since': time.monotonic(),
                                'last_plan': 0.0}
                objects, moving, _ = self.scene
                ids = sorted({obj.get('id', obj['name']) for obj in objects if obj['name'] in moving})
                self.report(f'WAITING for a moving obstacle ({", ".join(ids)}) '
                            f'to pass; {text}')
                return
        raise RuntimeError('moving obstacle is heading for the arm and there is no room on the path '
                           'to get out of its way; stopped. Clear its way and send the target again')

    def clear_while_waiting(self, trajectory, hold, names):
        """No collision while moving to the waiting pose and for WAIT_WINDOW s there,
        with obstacles a little early or late (TIME_MARGIN) as well."""
        until = seconds(trajectory.points[-1].time_from_start) - hold + WAIT_WINDOW
        return self.clear_with_margin(JointTrajectory(joint_names=names, points=[
            point for point in trajectory.points if seconds(point.time_from_start) <= until]), names)

    def clear_with_margin(self, trajectory, names):
        rows = np.array([point.positions for point in trajectory.points], dtype=float)
        times = times_of(trajectory)
        for shift in (0.0, TIME_MARGIN, -TIME_MARGIN):
            hit, by_moving = self.collision(rows, np.maximum(times + shift, 0.0), names)
            if hit is not None:
                self.get_logger().debug(f'not clear: obstacles {shift:+.1f} s, row {hit} of {len(rows)} '
                                        f'at +{times[hit]:.2f} s, moving={by_moving}')
                return False
        return True

    def wait_out(self, target, names, t_now):
        """Stopped for a moving obstacle: go on once the rest of the path is clear.

        Backs off further if the obstacle turns out to come at the arm where it
        waits. If the rest of the path is blocked by something standing -- the
        obstacle stopped on it -- plans a way around that from here instead.
        """
        runner, waiting = self.trajectory_runner, self.waiting
        waited = time.monotonic() - waiting['since']
        if waited > self.wait_limit():
            raise RuntimeError(f'still blocked after {waited:.1f} s of waiting and {self.replans} new '
                               'paths; stopped. Send the target again once the way is clear '
                               '(avoid.wait_before_replan, avoid.max_replans)')
        if t_now < waiting['rest_at']:
            return  # still getting to the waiting pose
        q = state_at(runner.sent, t_now)[0]
        count = max(1, int(self.avoid['horizon'] / runner.step))
        hit, by_moving = self.collision(np.repeat(q[None], count, axis=0),
                                        runner.step * np.arange(1, count + 1), names)
        if hit is not None and by_moving:
            self.back_further(names, waited)
            return
        rest = resume_from(waiting['path'], waiting['s_rest'], waiting['end'], self.avoid['stop_time'],
                           runner.step)
        if self.clear_with_margin(rest, names):
            waiting['clear'] += 1
            if waiting['clear'] >= 2:  # two readings in a row, not one lucky one
                runner.replace(rest)
                self.waiting = None
                self.report(f'RESUMING on the path after waiting {waited:.1f}s')
            return
        waiting['clear'] = 0
        hit, by_moving = self.trajectory_collision(rest, names)
        # A new path from here: at once when something standing blocks the way (the
        # obstacle stopped on it), after avoid.wait_before_replan when it stays blocked.
        blocked_too_long = time.monotonic() - waiting['blocked_since'] >= self.avoid['wait_before_replan']
        if hit is None or (by_moving and not blocked_too_long) or time.monotonic() - waiting['last_plan'] < 0.3:
            return  # still passing, or only just clear
        if self.replans >= self.avoid['max_replans']:
            raise RuntimeError(f'still blocked after {self.replans} new paths; stopped. Send the target '
                               'again once the way is clear (avoid.max_replans)')
        waiting['last_plan'] = time.monotonic()
        started = time.monotonic()
        self.plan_around(q, names)
        path, status = self.avoider.replan(q, np.zeros(len(names)), target, names)
        if path is None:
            if status.startswith('INVALID_START_STATE') and not waiting.get('backed_for_room'):
                # Stopped too close to the obstacle to plan from: back along the path it
                # came (known clear of what stands) once, then plan again from there.
                waiting['backed_for_room'] = True
                self.back_further(names, waited, why='too close to plan a way around from here')
                return
            if status in HOPELESS:
                raise RuntimeError(hopeless(status))
            return
        t_now = runner.now()
        new = splice(runner.sent, t_now, t_now + runner.step, path, self.metadata['joint_limits'],
                     runner.step, floor=self.detour_time(path, names))
        if not self.clear_with_margin(new, names):
            return
        runner.replace(new)
        self.waiting = None
        self.replans += 1
        self.report(f'RESUMING along a new path ({self.replans} of at most {self.avoid["max_replans"]}) '
                    f'after waiting {waited:.1f}s (replanned in {time.monotonic() - started:.2f}s)')

    def back_further(self, names, waited, why='the obstacle comes at the arm'):
        """The obstacle comes at the arm where it waits (or is too close to plan from): back
        further along the path."""
        runner, waiting = self.trajectory_runner, self.waiting
        hold = max(self.wait_limit() - waited, 0.0) + 2.0
        first = times_of(waiting['path'])[0]
        for back in BACK_OFFS:
            s_rest = max(waiting['s_rest'] - back, first)
            if s_rest >= waiting['s_rest'] - 1e-6:
                break
            trajectory = back_off(waiting['path'], waiting['s_rest'], s_rest, hold, runner.step,
                                  limits=self.metadata['joint_limits'])
            if self.clear_while_waiting(trajectory, hold, names):
                runner.replace(trajectory)
                waiting.update(s_rest=s_rest, clear=0,
                               rest_at=seconds(trajectory.points[-1].time_from_start) - hold)
                self.report(f'WAITING: {why}; backing {back:.2f}s further along the path')
                return
        raise RuntimeError(f'{why}, and there is no room on the path to back away; stopped. Clear the way '
                           'and send the target again')

    def wait_limit(self):
        """Longest a stop for an obstacle can last: every new path given its wait, and one more."""
        return self.avoid['wait_before_replan'] * (self.avoid['max_replans'] + 1)

    def retarget(self, names, t_now):
        """A newer target while moving: plan toward it from where the arm will be
        avoid.lead from now, at the speed it will have there, and take over without a
        stop. Stopped and waiting: plan from where it stands. Returns the new target."""
        runner = self.trajectory_runner
        target, self.pending = self.pending, None
        self.report('REPLACED by a newer target')
        status = 'not tried'
        for _ in range(4):  # a plan that finishes after its join time is planned again
            started = time.monotonic()
            if self.waiting is not None:
                t_join = t_now + runner.step
                q, v = state_at(runner.sent, t_now)[0], np.zeros(len(names))
            else:
                t_join = min(t_now + self.avoid['lead'], runner.motion_end())
                q, v = state_at(runner.sent, t_join)
            self.plan_around(q, names)
            path, status = self.avoider.replan(q, v, target, names)
            if path is None and status in HOPELESS:
                break
            t_now = runner.now()
            if path is None or t_now >= t_join - runner.step:
                continue
            new = splice(runner.sent, t_now, t_join, path, self.metadata['joint_limits'],
                         runner.step, floor=self.detour_time(path, names))
            runner.replace(new)
            self.waiting = None
            self.replans = 0
            self.report(f'EXECUTING toward the new target (replanned in {time.monotonic() - started:.2f}s, '
                        f'takes over {t_join - t_now:.2f}s ahead)')
            return target
        if status in HOPELESS:
            raise RuntimeError(f'new target: {hopeless(status)}')
        return self.retarget_from_standing(target, status)

    def retarget_from_standing(self, target, status):
        """No path to the new target from the moving arm -- the replanner is tuned for
        short detours and can miss from a fast start. Stop on the current path, then
        plan with cuMotion from standing, as for any target."""
        runner = self.trajectory_runner
        hold = self.wait_limit() + 2.0
        stop, _ = slow_to_stop(runner.sent, runner.now(), self.avoid['stop_time'], hold, runner.step)
        runner.replace(stop)
        self.waiting = None
        self.report(f'WAITING: no path to the new target from the moving arm ({status}); stopping to plan '
                    'from standing')
        rest_at = seconds(stop.points[-1].time_from_start) - hold
        while runner.now() < rest_at and not runner.done():
            rclpy.spin_once(self, timeout_sec=self.avoid['check_period'])
        self.report('PLANNING the new target from standing')
        planned = self.measure(target=target)
        path = planned['trajectory'].joint_trajectory
        duration, reason = self.move_duration(path)
        trajectory, load = retime(path, duration, runner.step, self.metadata['joint_limits'])
        runner.replace(trajectory)
        self.replans = 0
        self.report(f'EXECUTING toward the new target planner_time={planned["planner_time"]:.3f}s '
                    f'duration={duration:.2f}s ({reason}) peak_velocity={load * 100:.0f}%')
        return target

    def detour_time(self, path, names):
        """The replanned path at cuRobo's own pace, slowed only as the motion limits require.

        Not stretched to the rest of the move it replaces: getting out of a
        moving obstacle's way depends on moving when the plan says to.
        """
        native = (len(path) - 1) * PLAN_DT
        timed = JointTrajectory(joint_names=list(names))
        for index, row in enumerate(path):
            timed.points.append(JointTrajectoryPoint(positions=list(map(float, row)),
                                                     time_from_start=stamp(index * PLAN_DT)))
        needed, _ = motion_duration(timed, self.chain, self.snapshot(), self.metadata['joint_limits'],
                                    self.param('linear_velocity_limit'),
                                    self.param('angular_velocity_limit'), 0.0)
        return max(native, needed)

    def on_target(self, msg):
        try:
            target = check_transform(msg.data)
        except ValueError as error:
            self.report(f'FAILED: rejected target: {error}')
            return
        if self.busy:
            self.get_logger().info('new target: it replaces the one in progress' if self.avoider is not None
                                   else 'new target: it runs when the current move ends (avoid.enabled is off)')
        self.pending, self.pending_at = target, time.monotonic()  # only the latest counts

    def process(self, target):
        self.busy = True
        try:
            self.report('PLANNING')
            self.sync_attached(wait=True)
            problem = robot_problem(self.robot_state)
            if problem:
                raise RuntimeError(problem)
            planned = self.measure(target=target)
            path = planned['trajectory'].joint_trajectory
            duration, reason = self.move_duration(path)
            trajectory, load = retime(path, duration, self.param('step'),
                                      self.metadata['joint_limits'])
            self.report(f'EXECUTING planner_time={planned["planner_time"]:.3f}s '
                        f'duration={duration:.2f}s ({reason}) steps={len(trajectory.points)} '
                        f'peak_velocity={load * 100:.0f}%')
            runner = self.trajectory_runner
            runner.step, runner.hold = self.param('step'), self.param('hold')
            runner.endpoint_tolerance = self.param('endpoint_tolerance')
            if self.avoider is None:
                runner.execute(trajectory, self.snapshot)
            else:
                self.execute_avoiding(trajectory, target)
            self.report('DONE')
        except Exception as error:
            self.report(f'FAILED: {error}')
        finally:
            self.busy = False

    def warm_replanner(self, attempts=3):
        """Replan once from a moving start before the first real one needs to be quick.

        The first replan of a process runs code paths the MotionGen warmup does not
        (a start velocity, a new goal): it took 0.27 s, longer than avoid.lead, and
        the arm stopped for an obstacle it had time to get around.
        """
        names = list(self.metadata['active_joints'])
        current = self.fresh_snapshot()
        q = np.array([current[name] for name in names])
        target = self.chain.pose(current)
        target[:3, 3] += [0.03, 0.0, 0.0]
        self.avoider.plan_world([])
        started = time.monotonic()
        for _ in range(attempts):
            path, status = self.avoider.replan(q, np.full(len(names), 0.05), target, names)
            if path is not None:
                break
        self.get_logger().info(f'replanner warmed up in {time.monotonic() - started:.2f}s '
                               f'({"ok" if path is not None else status})')

    def run(self):
        self.check_robot()
        self.prepare()
        if self.avoider is not None:
            self.warm_replanner()
        if self.tracking_cfg['prepare']:
            self.build_tracker()
        self.trajectory_runner.connect()
        self.get_logger().info(f'waiting for 4x4 targets on {self.param("target_topic")} '
                               f'(group={self.metadata["group"]}, tool={self.metadata["tool_frame"]}, '
                               f'frame={self.metadata["base_frame"]})')
        self.report('READY')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self.sync_attached()
            except Exception as error:  # reported; planning goes on without the modules
                self.get_logger().error(f'attached modules: {error}')
            if self.tracking:
                try:
                    self.track()
                except Exception as error:
                    self.report(f'FAILED: tracking: {error}')
                continue
            if self.pending is not None:
                target, self.pending = self.pending, None
                self.process(target)

    def cancel_active_goal(self):
        super().cancel_active_goal()
        self.trajectory_runner.cancel()


def main(args=None):
    run_node(TargetExecutor)


if __name__ == '__main__':
    main()
