#include "rby1_apriltag/hand_target.hpp"

namespace rby1_apriltag {

Eigen::Vector3d hand_point(const Eigen::Isometry3d & frame_from_camera, const Eigen::Vector3d & marker,
                           const Eigen::Vector3d & offset) {
  return frame_from_camera * marker + offset;
}

std::array<double, 16> target_values(const Eigen::Matrix3d & orientation, const Eigen::Vector3d & point) {
  std::array<double, 16> values{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) values[row * 4 + column] = orientation(row, column);
    values[row * 4 + 3] = point[row];
  }
  values[15] = 1.0;
  return values;
}

}  // namespace rby1_apriltag
