"""Initialize the local Docker simulator through the real RBY1 ROS driver."""

import json
import math
from pathlib import Path
import subprocess
import time

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.srv import StateOnOff
from sensor_msgs.msg import JointState

from rby1_cumotion.model import load_model


def validate_simulator(info, model):
    """Require our M v1.2 container and its loopback-only SDK port."""
    if model != 'm_1_2':
        raise ValueError('The Docker example supports only the M v1.2 simulator')
    if not info['State']['Running'] or info['Config']['Image'] != 'rby1_cumotion_sim:m-v1.2':
        raise ValueError('Start the rby1_cumotion_sim:m-v1.2 container first')
    bindings = info['NetworkSettings']['Ports'].get('50051/tcp')
    if bindings != [{'HostIp': '127.0.0.1', 'HostPort': '50051'}]:
        raise ValueError('Simulator SDK port must be published as 127.0.0.1:50051:50051')


def check_simulator(container, model):
    result = subprocess.run(['docker', 'inspect', '--type', 'container', container],
                            check=True, capture_output=True, text=True, timeout=10)
    validate_simulator(json.loads(result.stdout)[0], model)


class PrepareSim(Node):
    def __init__(self):
        super().__init__('rby1_prepare_sim')
        self.declare_parameter('model_directory', '')
        self.declare_parameter('group', '')
        self.declare_parameter('container', 'rby1-cumotion-sim')
        self.declare_parameter('driver_namespace', 'rby1')
        self.metadata, _ = load_model(Path(self.get_parameter('model_directory').value),
                                      self.get_parameter('group').value or None)
        check_simulator(self.get_parameter('container').value, self.metadata['model'])
        self.prefix = '/' + self.get_parameter('driver_namespace').value.strip('/')
        self.positions = {}
        self.received = 0.0
        self.stamp = 0.0
        self.goal_handle = None
        self.subscription = self.create_subscription(
            JointState, self.prefix + '/joint_states', self.on_state, qos_profile_sensor_data)

    def on_state(self, msg):
        if len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name):
            self.positions = dict(zip(msg.name, msg.position))
            self.received = time.monotonic()
            self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('Simulator driver request timed out')
        return future.result()

    def call(self, service, kind, request):
        client = self.create_client(kind, self.prefix + '/' + service)
        try:
            if not client.wait_for_service(timeout_sec=30.0):
                raise TimeoutError(f'Driver service unavailable: {service}')
            result = self.wait(client.call_async(request))
            if hasattr(result, 'success') and not result.success:
                raise RuntimeError(f'{service}: {result.message}')
            return result
        finally:
            self.destroy_client(client)

    def run(self):
        params = self.call('rby1_ros2_driver/get_parameters', GetParameters,
                           GetParameters.Request(names=['robot_ip', 'model']))
        if [value.string_value for value in params.values] != ['127.0.0.1:50051', 'm']:
            raise ValueError('Driver must connect to the local M simulator at 127.0.0.1:50051')
        for service in ('robot_power', 'robot_servo'):
            self.call(service, StateOnOff, StateOnOff.Request(state=True, parameters='all'))
        client = ActionClient(self, Rby1JointCommand, self.prefix + '/robot_joint')
        try:
            if not client.wait_for_server(timeout_sec=30.0):
                raise TimeoutError('Driver robot_joint action unavailable')
            goal = Rby1JointCommand.Goal()
            for group, count in [('torso', 6), ('right_arm', 7), ('left_arm', 7), ('head', 2)]:
                command = getattr(goal, group)
                command.joint_names = [f'{group}_{i}' for i in range(count)]
                command.position = [self.metadata['default_positions'][n] for n in command.joint_names]
                command.minimum_time = 5.0
                command.velocity_limit = 0.5
                command.acceleration_limit = 0.5
            self.get_logger().info('Moving the Docker simulator to the model posture through /robot_joint')
            self.goal_handle = self.wait(client.send_goal_async(goal), 10.0)
            if not self.goal_handle.accepted:
                raise RuntimeError('Simulator posture goal rejected; stop existing controllers first')
            result = self.wait(self.goal_handle.get_result_async())
            if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
                raise RuntimeError(f'Simulator posture failed: {result.result.finish_code}')
            self.goal_handle = None
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = self.get_clock().now().nanoseconds * 1e-9
                if (time.monotonic() - self.received < 1.0 and abs(now - self.stamp) < 1.0
                        and all(n in self.positions and math.isfinite(self.positions[n])
                                and abs(self.positions[n] - q) < 0.01
                                for n, q in self.metadata['default_positions'].items())):
                    self.get_logger().info('SIM_READY: all 22 measured joints match the model posture')
                    return
            raise RuntimeError('Simulator feedback did not reach the model posture')
        finally:
            if self.goal_handle and self.goal_handle.accepted:
                self.wait(self.goal_handle.cancel_goal_async(), 5.0)
            client.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = PrepareSim()
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        code = 1
        print(f'SIM_INITIALIZATION_FAILED: {error}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)
