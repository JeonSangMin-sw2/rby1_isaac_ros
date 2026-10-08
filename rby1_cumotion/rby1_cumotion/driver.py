"""Talking to the RB-Y1 ROS driver that runs on the host.

The driver is the robot here, simulator or real alike: its robot_ip decides which
one it drives, and nothing on this side changes. Everything goes through the
driver's own services and actions, never through ros2_control -- the driver
refuses follow_joint_trajectory while ros2_control's hardware plugin holds it.
"""

import os
import time

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import RobotState
from rby1_msgs.srv import StateOnOff


class Driver:
    """Service and action access to one driver instance, by its namespace."""

    def __init__(self, node, namespace='rby1', timeout=30.0):
        self.node = node
        self.prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
        self.timeout = timeout

    def name(self, relative):
        return f'{self.prefix}/{relative}'

    def wait(self, future, timeout=None):
        end = time.monotonic() + (timeout or self.timeout)
        while rclpy.ok() and not future.done():
            if time.monotonic() >= end:
                raise TimeoutError('Driver request timed out')
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError('ROS shut down while waiting for the driver')
        return future.result()

    def call(self, service, kind, request):
        client = self.node.create_client(kind, self.name(service))
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise TimeoutError(f'Driver service unavailable: {self.name(service)} '
                                   '-- is the driver running on the host, same ROS domain?')
            result = self.wait(client.call_async(request))
            if hasattr(result, 'success') and not result.success:
                raise RuntimeError(f'{service}: {result.message}')
            return result
        finally:
            self.node.destroy_client(client)

    def switch(self, service, state, parameters='all'):
        return self.call(service, StateOnOff, StateOnOff.Request(state=state, parameters=parameters))

    def action(self, kind, relative):
        client = ActionClient(self.node, kind, self.name(relative))
        if not client.wait_for_server(timeout_sec=self.timeout):
            client.destroy()
            raise TimeoutError(f'Driver action unavailable: {self.name(relative)}')
        return client


# Elbow bent 90 deg, arm forward: away from the straight-arm singularity, and
# the posture the tutorial's example coordinates are written for.
READY = {
    'right_arm': [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0],
    'left_arm': [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0],
}


def straight_arms(positions, group_joints, threshold):
    """Arms of the planning group whose elbow (joint 3) is within `threshold` of straight."""
    return [arm for arm in READY
            if f'{arm}_3' in group_joints and abs(positions[f'{arm}_3']) < threshold]


def ready_goal(arms, minimum_time):
    """A Rby1JointCommand moving only `arms` to READY; other parts are not commanded."""
    goal = Rby1JointCommand.Goal()
    for arm in arms:
        command = getattr(goal, arm)
        command.position = list(READY[arm])
        command.minimum_time = minimum_time
    return goal


def robot_problem(state):
    """Why the robot cannot take motion now, from a RobotState, or None."""
    if state.emo_state:
        return 'Emergency stop is pressed; release it and start again'
    if state.control_manager_state == RobotState.STATE_MAJOR_FAULT:
        return ('Control manager is in a major fault. Reset it with: ros2 service call '
                '/rby1/control_manager_command rby1_msgs/srv/ControlManagerCommand '
                '"{command: 3}"  (CMD_RESET), then start again')
    return None


def check_driver_model(driver_model, bundle_model):
    """The driver knows only the robot kind (m/a); refuse a bundle for the other one."""
    kind = bundle_model.split('_', 1)[0]
    if driver_model.strip().lower() != kind:
        raise ValueError(f'Driver runs an RB-Y1 {driver_model!r} but the planner was started '
                         f'for {bundle_model}; restart the cuMotion launch against this driver')


def bundle_name(kind, version):
    """'m' and 1.2 -> 'm_1_2', the directory name under $RBY1_BUNDLES."""
    kind = kind.strip().lower()
    if kind not in ('m', 'a'):
        raise ValueError(f'Driver reports robot kind {kind!r}; expected m or a')
    if not version > 0:
        raise ValueError(
            f'Driver reports robot_version {version}. Drivers before the fix in '
            'rby1_ros2_driver.cpp publish 0.0 because the SDK says "v1.2". Stop the driver, '
            'rebuild it (cd ~/ros2_driver_ws && colcon build --packages-select rby1_driver) and '
            'start it again -- or name the model in the launch (model:=m_1_2).')
    return f'{kind}_{version:.1f}'.replace('.', '_')


def read_robot(namespace='rby1', timeout=15.0):
    """Robot kind, version and joint positions, read from a running driver.

    Runs in its own context so it can be called from a launch file, which may
    already own rclpy's default one.
    """
    context = rclpy.Context()
    # args=[]: rclpy otherwise parses sys.argv, which in a launch file holds the
    # launch arguments ('hardware:=driver') and reads them as ROS remap rules.
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('rby1_cumotion_robot_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    seen = {}
    node.create_subscription(JointState, f'{prefix}/joint_states',
                             lambda m: seen.setdefault('positions', dict(zip(m.name, m.position))),
                             qos_profile_sensor_data)
    node.create_subscription(RobotState, f'{prefix}/robot_state',
                             lambda m: seen.setdefault('version', m.robot_version), 10)
    client = node.create_client(GetParameters, f'{prefix}/rby1_ros2_driver/get_parameters')
    try:
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'No RB-Y1 driver answers under {prefix or "/"} -- start it on the '
                               'host (ros2 launch rby1_driver rby1_ros2_driver.launch.py) on '
                               'the same ROS_DOMAIN_ID')
        future = client.call_async(GetParameters.Request(names=['model']))
        while not (future.done() and 'positions' in seen and 'version' in seen):
            if time.monotonic() > deadline:
                hint = ''
                if os.geteuid() == 0:
                    # Seen for real: the driver's service answered, its topics never came.
                    hint = (' You are root: ROS 2 shared-memory transport cannot take data '
                            'from host processes run by another user. Enter the container as '
                            'admin (docker exec -it -u admin ...).')
                raise TimeoutError(f'Driver under {prefix or "/"} did not publish joint_states '
                                   f'and robot_state in time.{hint}')
            executor.spin_once(timeout_sec=0.05)
        kind = future.result().values[0].string_value
        return kind, seen['version'], seen['positions']
    finally:
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
