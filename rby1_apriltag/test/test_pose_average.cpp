// Marker pose smoothing; no ROS graph.
#include <gtest/gtest.h>

#include <cmath>
#include <deque>

#include "rby1_apriltag/pose_average.hpp"

using rby1_apriltag::robust_average;

namespace {

Eigen::Isometry3d pose(double x, double y, double z, double yaw) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

}  // namespace

TEST(RobustAverage, OneWildFrameDoesNotDragTheTranslation) {
  std::deque<Eigen::Isometry3d> frames{pose(0.0, 0.0, 0.32, 0), pose(0.001, 0.0, 0.321, 0),
                                       pose(0.5, 0.4, 2.0, 0), pose(-0.001, 0.0, 0.319, 0),
                                       pose(0.0, 0.001, 0.32, 0)};
  const auto mean = robust_average(frames);
  EXPECT_NEAR(mean.translation().z(), 0.32, 1e-12);
  EXPECT_NEAR(mean.translation().x(), 0.0, 1e-12);
}

TEST(RobustAverage, EvenCountTakesTheMiddleTwo) {
  std::deque<Eigen::Isometry3d> frames{pose(0, 0, 1.0, 0), pose(0, 0, 2.0, 0), pose(0, 0, 3.0, 0),
                                       pose(0, 0, 10.0, 0)};
  EXPECT_NEAR(robust_average(frames).translation().z(), 2.5, 1e-12);  // numpy.median
}

TEST(RobustAverage, RotationIsTheMeanAndProper) {
  std::deque<Eigen::Isometry3d> frames{pose(0, 0, 0, 0.10), pose(0, 0, 0, 0.20), pose(0, 0, 0, 0.30)};
  const auto mean = robust_average(frames);
  EXPECT_NEAR(Eigen::AngleAxisd(mean.rotation()).angle(), 0.20, 1e-9);
  EXPECT_NEAR(mean.rotation().determinant(), 1.0, 1e-12);
  EXPECT_TRUE((mean.rotation() * mean.rotation().transpose()).isIdentity(1e-12));
}

TEST(RobustAverage, NothingToAverageIsRefused) {
  EXPECT_THROW(robust_average({}), std::invalid_argument);
}

TEST(DetectionFrame, IsaacLeavesThePoseFrameEmpty) {
  EXPECT_EQ(rby1_apriltag::detection_frame("", "camera_optical_frame"), "camera_optical_frame");
  EXPECT_EQ(rby1_apriltag::detection_frame("tag_camera", "camera_optical_frame"), "tag_camera");
}
