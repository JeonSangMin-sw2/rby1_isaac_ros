#include "rby1_apriltag/pose_average.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

#include <Eigen/SVD>

namespace rby1_apriltag {

namespace {

double median(std::vector<double> values) {
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  if (values.size() % 2) return values[middle];
  const double upper = values[middle];
  const double lower = *std::max_element(values.begin(), values.begin() + middle);
  return (lower + upper) / 2.0;  // as numpy.median for an even count
}

}  // namespace

Eigen::Isometry3d robust_average(const std::deque<Eigen::Isometry3d> & transforms) {
  if (transforms.empty()) throw std::invalid_argument("nothing to average");
  Eigen::Vector3d translation;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    for (const auto & transform : transforms) values.push_back(transform.translation()[axis]);
    translation[axis] = median(values);
  }
  Eigen::Matrix3d sum = Eigen::Matrix3d::Zero();
  for (const auto & transform : transforms) sum += transform.rotation();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU();
  Eigen::Matrix3d rotation = u * svd.matrixV().transpose();
  if (rotation.determinant() < 0.0) {
    u.col(2) *= -1.0;
    rotation = u * svd.matrixV().transpose();
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation;
  result.translation() = translation;
  return result;
}

std::string detection_frame(const std::string & pose_frame, const std::string & array_frame) {
  return pose_frame.empty() ? array_frame : pose_frame;
}

}  // namespace rby1_apriltag
