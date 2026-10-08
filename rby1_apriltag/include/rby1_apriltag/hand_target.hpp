// A marker the camera sees, as a target for a hand.
#pragma once

#include <array>

#include <Eigen/Geometry>

namespace rby1_apriltag {

// Where the hand goes for a marker: the marker (a point in the camera's frame) taken
// into the target frame, then moved by `offset` along the target frame's axes.
Eigen::Vector3d hand_point(const Eigen::Isometry3d & frame_from_camera, const Eigen::Vector3d & marker,
                           const Eigen::Vector3d & offset);

// What the target executors take: the 4x4 transform of the tool frame, row-major.
std::array<double, 16> target_values(const Eigen::Matrix3d & orientation, const Eigen::Vector3d & point);

}  // namespace rby1_apriltag
