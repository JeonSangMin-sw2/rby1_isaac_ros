#!/usr/bin/env python3
"""
Target AprilTag Filter & Pose Stabilizer Node.

Filters specific AprilTag IDs from Isaac ROS cuAprilTag detections,
stabilizes the 6-DoF pose using robust forward-matrix median & SVD averaging,
and publishes standard ROS 2 topics and TF frames for downstream host applications.
"""

from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

try:
    from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert a quaternion into a 3x3 rotation matrix."""
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    else:
        return np.eye(3, dtype=np.float64)

    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)]
    ], dtype=np.float64)


def rot_matrix_to_quat(R: np.ndarray):
    """Convert a 3x3 rotation matrix into a normalized quaternion (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    else:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

    return qx, qy, qz, qw


def robust_average_forward_transforms(transforms: list):
    """
    Robust transform averaging on Forward (Camera-to-Marker) transforms:
    - Translation: Component-wise median
    - Rotation: SVD-based orthogonal mean
    """
    # 1. Median for translation
    translations = np.array([t[0:3, 3] for t in transforms], dtype=np.float64)
    avg_trans = np.median(translations, axis=0)

    # 2. SVD rotation averaging
    sum_R = np.sum([t[0:3, 0:3] for t in transforms], axis=0)
    U, _, Vt = np.linalg.svd(sum_R)
    final_R = U @ Vt
    if np.linalg.det(final_R) < 0.0:
        U[:, 2] *= -1.0
        final_R = U @ Vt

    qx, qy, qz, qw = rot_matrix_to_quat(final_R)
    return avg_trans, (qx, qy, qz, qw)


class TargetTagFilter(Node):
    def __init__(self):
        super().__init__('target_tag_filter')

        # Parameter Declarations
        self.declare_parameter('target_ids', [7])
        self.declare_parameter('input_topic', '/tag_detections')
        self.declare_parameter('output_pose_topic', '/target_marker/pose')
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('target_frame_prefix', 'target_marker')
        self.declare_parameter('filter_jitter', True)
        self.declare_parameter('window_size', 5)
        self.declare_parameter('publish_per_tag_topics', True)

        # Get Parameters
        target_ids_param = self.get_parameter('target_ids').value
        self.target_ids = set(target_ids_param)
        self.input_topic = self.get_parameter('input_topic').value
        self.output_pose_topic = self.get_parameter('output_pose_topic').value
        self.broadcast_tf = self.get_parameter('broadcast_tf').value
        self.target_frame_prefix = self.get_parameter('target_frame_prefix').value
        self.filter_jitter = self.get_parameter('filter_jitter').value
        self.window_size = max(1, self.get_parameter('window_size').value)
        self.publish_per_tag_topics = self.get_parameter('publish_per_tag_topics').value

        # Buffers for sliding-window transform averaging: {tag_id: deque(maxlen=window_size)}
        self.history = {}

        # Check interface availability
        if AprilTagDetectionArray is None:
            self.get_logger().error(
                "Could not import 'isaac_ros_apriltag_interfaces.msg.AprilTagDetectionArray'. "
                "Make sure 'isaac_ros_apriltag_interfaces' is built and sourced in your environment."
            )
            return

        # Publishers
        self.pub_pose = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.per_tag_pubs = {}
        if self.publish_per_tag_topics:
            for tag_id in self.target_ids:
                topic_name = f"/target_marker_{tag_id}/pose"
                self.per_tag_pubs[tag_id] = self.create_publisher(PoseStamped, topic_name, 10)

        # TF Broadcaster
        if self.broadcast_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            self.input_topic,
            self.tag_callback,
            10
        )

        self.get_logger().info(
            f"TargetTagFilter initialized. Filtering target_ids: {sorted(list(self.target_ids))}, "
            f"Jitter filter: {'Enabled (window=' + str(self.window_size) + ')' if self.filter_jitter else 'Disabled'}"
        )

    def tag_callback(self, msg: AprilTagDetectionArray):
        for detection in msg.detections:
            tag_id = detection.id
            if tag_id not in self.target_ids:
                continue

            raw_pose = detection.pose.pose.pose
            qx = raw_pose.orientation.x
            qy = raw_pose.orientation.y
            qz = raw_pose.orientation.z
            qw = raw_pose.orientation.w
            tx = raw_pose.position.x
            ty = raw_pose.position.y
            tz = raw_pose.position.z

            if self.filter_jitter:
                # Construct 4x4 forward transform matrix
                R = quat_to_rot_matrix(qx, qy, qz, qw)
                T = np.eye(4, dtype=np.float64)
                T[0:3, 0:3] = R
                T[0:3, 3] = [tx, ty, tz]

                if tag_id not in self.history:
                    self.history[tag_id] = deque(maxlen=self.window_size)
                self.history[tag_id].append(T)

                # Compute robust average on forward matrices
                avg_trans, (qx, qy, qz, qw) = robust_average_forward_transforms(list(self.history[tag_id]))
                tx, ty, tz = avg_trans[0], avg_trans[1], avg_trans[2]

            # 1. Publish standard PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.header.frame_id = detection.pose.header.frame_id
            pose_msg.pose.position.x = float(tx)
            pose_msg.pose.position.y = float(ty)
            pose_msg.pose.position.z = float(tz)
            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)

            self.pub_pose.publish(pose_msg)

            if self.publish_per_tag_topics:
                if tag_id not in self.per_tag_pubs:
                    self.per_tag_pubs[tag_id] = self.create_publisher(
                        PoseStamped, f"/target_marker_{tag_id}/pose", 10
                    )
                self.per_tag_pubs[tag_id].publish(pose_msg)

            # 2. Broadcast TF
            if self.broadcast_tf:
                t = TransformStamped()
                t.header = msg.header
                t.header.frame_id = detection.pose.header.frame_id
                t.child_frame_id = f"{self.target_frame_prefix}_{tag_id}"
                t.transform.translation.x = float(tx)
                t.transform.translation.y = float(ty)
                t.transform.translation.z = float(tz)
                t.transform.rotation.x = float(qx)
                t.transform.rotation.y = float(qy)
                t.transform.rotation.z = float(qz)
                t.transform.rotation.w = float(qw)
                self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTagFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
