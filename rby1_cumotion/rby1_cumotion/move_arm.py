"""Plan a small Cartesian displacement, then optionally execute through MoveIt."""

import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint, PositionConstraint
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from rby1_cumotion.model import forward_kinematics, load_model


def check_locked(metadata, positions, tolerance):
    for name, expected in metadata['locked_joints'].items():
        if name not in positions or not math.isfinite(positions[name]):
            raise ValueError(f'Missing/nonfinite locked joint: {name}')
        if abs(positions[name] - expected) > tolerance:
            raise ValueError(f'Locked joint {name} is {positions[name]:.4f}; model expects {expected:.4f}. '
                             'Prepare a model with the current fixed posture before planning.')


def validate_trajectory(trajectory, metadata, current, tolerance=0.03):
    names = list(trajectory.joint_names)
    if len(names) != 7 or set(names) != set(metadata['active_joints']):
        raise ValueError('Planner returned joints outside the selected arm, or missing/duplicate joints')
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


class MoveArm(Node):
    def __init__(self):
        super().__init__('rby1_cumotion_example')
        for name, default in {
            'model_directory': '', 'pipeline': 'isaac_ros_cumotion', 'execute': False,
            'offset_xyz': [0.03, 0.0, 0.0], 'velocity_scaling': 0.1,
            'acceleration_scaling': 0.1, 'planning_time': 30.0,
            'server_timeout': 120.0, 'state_max_age': 2.0,
            'locked_joint_tolerance': 0.01, 'joint_states_topic': '/joint_states',
            'move_action': '/move_action', 'execute_action': '/execute_trajectory',
        }.items():
            self.declare_parameter(name, default)
        self.metadata, self.robot = load_model(Path(self.param('model_directory')))
        self.state = {}
        self.subscription = self.create_subscription(
            JointState, self.param('joint_states_topic'), self.on_state, qos_profile_sensor_data)
        self.planner = ActionClient(self, MoveGroup, self.param('move_action'))
        self.executor_client = ActionClient(self, ExecuteTrajectory, self.param('execute_action'))
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
            raise RuntimeError(f'Action failed: status={result.status}, MoveIt error={result.result.error_code.val}')
        return result.result

    def run(self):
        offset = self.param('offset_xyz')
        if len(offset) != 3 or not all(map(math.isfinite, offset)) or np.linalg.norm(offset) > 0.1:
            raise ValueError('offset_xyz must contain 3 finite metres, with norm <= 0.1')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            if not 0.0 < self.param(name) <= 1.0:
                raise ValueError(f'{name} must be in (0, 1]')
        for name in ('planning_time', 'server_timeout', 'state_max_age', 'locked_joint_tolerance'):
            if not math.isfinite(self.param(name)) or self.param(name) <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.param('pipeline') not in ('isaac_ros_cumotion', 'ompl'):
            raise ValueError('pipeline must be isaac_ros_cumotion or ompl')
        if not self.planner.wait_for_server(timeout_sec=self.param('server_timeout')):
            raise TimeoutError('MoveIt move_action server is unavailable')
        if self.param('execute') and not self.executor_client.wait_for_server(timeout_sec=self.param('server_timeout')):
            raise TimeoutError('MoveIt execute_trajectory server is unavailable')
        end = time.monotonic() + 10.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                positions = self.snapshot()
                break
            except ValueError:
                if time.monotonic() >= end:
                    raise
        tip = forward_kinematics(self.robot, positions, self.metadata['tool_frame'])
        target = Pose()
        target.position.x, target.position.y, target.position.z = (tip[:3, 3] + offset).tolist()
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = quaternion(tip[:3, :3]).tolist()
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
        begin = time.monotonic()
        result = self.run_action(self.planner, goal, self.param('planning_time') + 30.0)
        current = self.snapshot()
        trajectory = result.planned_trajectory
        validate_trajectory(trajectory.joint_trajectory, self.metadata, current)
        duration = trajectory.joint_trajectory.points[-1].time_from_start
        seconds = duration.sec + duration.nanosec * 1e-9
        self.get_logger().info(f'PLAN_OK pipeline={request.pipeline_id} group={request.group_name} '
                               f'planner_time={result.planning_time:.3f}s wall_time={time.monotonic()-begin:.3f}s '
                               f'points={len(trajectory.joint_trajectory.points)} duration={seconds:.3f}s')
        if self.param('execute'):
            self.snapshot()
            execution = ExecuteTrajectory.Goal(trajectory=trajectory)
            self.run_action(self.executor_client, execution, seconds + 30.0)
            current = self.snapshot()
            final = trajectory.joint_trajectory.points[-1].positions
            if any(abs(current[n] - q) > 0.03 for n, q in zip(trajectory.joint_trajectory.joint_names, final)):
                raise RuntimeError('Controller succeeded but measured final state differs from trajectory')
            self.get_logger().info('EXECUTION_OK: measured arm reached the planned endpoint')


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = MoveArm()
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        exit_code = 1
        if node:
            node.get_logger().error(str(error) or 'Interrupted')
            if node.active_goal and node.active_goal.accepted:
                try:
                    response = node.wait(node.active_goal.cancel_goal_async(), 5.0)
                    if not response.goals_canceling:
                        node.get_logger().error('Action cancellation was not acknowledged')
                except Exception as cancel_error:
                    node.get_logger().error(f'Action cancellation failed: {cancel_error}')
        else:
            print(f'RBY1 example failed: {error}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
