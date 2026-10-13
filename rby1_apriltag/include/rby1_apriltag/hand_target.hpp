// A marker the camera sees, as a target for a hand.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace rby1_apriltag {

// Where the hand goes for a marker: the marker (a point in the camera's frame) taken
// into the target frame, then moved by `offset` along the target frame's axes.
Eigen::Vector3d hand_point(const Eigen::Isometry3d & frame_from_camera, const Eigen::Vector3d & marker,
                           const Eigen::Vector3d & offset);

// What the target executors take: the 4x4 transform of the tool frame, row-major.
std::array<double, 16> target_values(const Eigen::Matrix3d & orientation, const Eigen::Vector3d & point);

// The marker a round over the markers in view goes to next: the lowest id above `last`,
// or the lowest of all when none is above it (or nothing was sent yet). `seen` in any
// order; nullopt when it is empty.
std::optional<int64_t> next_marker(const std::vector<int64_t> & seen, std::optional<int64_t> last);

}  // namespace rby1_apriltag
