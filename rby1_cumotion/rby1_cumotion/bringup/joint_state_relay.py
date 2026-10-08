"""Republish the driver's joint states on /joint_states.

The RB-Y1 driver publishes under its namespace (/rby1/joint_states). MoveIt,
robot_state_publisher and this package's nodes all read /joint_states, and with mock
hardware that is where joint_state_broadcaster publishes. Relaying here keeps
every downstream node identical across hardware:=mock and hardware:=driver.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__('rby1_joint_state_relay')
        self.declare_parameter('source', '/rby1/joint_states')
        self.declare_parameter('target', '/joint_states')
        source = self.get_parameter('source').value
        target = self.get_parameter('target').value
        if source == target:
            raise ValueError('source and target must differ, or the relay feeds itself')
        self.publisher = self.create_publisher(JointState, target, 10)
        self.subscription = self.create_subscription(
            JointState, source, self.publisher.publish, qos_profile_sensor_data)
        self.get_logger().info(f'relaying {source} -> {target}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
