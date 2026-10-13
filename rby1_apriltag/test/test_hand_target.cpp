// A marker as a hand target; no ROS graph.
#include <gtest/gtest.h>

#include <cmath>

#include "rby1_apriltag/hand_target.hpp"

using rby1_apriltag::hand_point;
using rby1_apriltag::target_values;

TEST(HandPoint, TheOffsetIsAlongTheTargetFramesAxes) {
  // The camera looks along base x: its optical z is base x, its x is base -y, its y is base -z.
  Eigen::Isometry3d base_from_camera = Eigen::Isometry3d::Identity();
  base_from_camera.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  base_from_camera.translation() = Eigen::Vector3d(0.1, 0.0, 1.5);
  // A marker 0.5 m in front of the camera, 0.2 m to its right.
  const auto point = hand_point(base_from_camera, Eigen::Vector3d(0.2, 0.0, 0.5), Eigen::Vector3d(0.0, 0.0, -0.10));
  EXPECT_NEAR(point.x(), 0.6, 1e-12);
  EXPECT_NEAR(point.y(), -0.2, 1e-12);
  EXPECT_NEAR(point.z(), 1.4, 1e-12);  // 10 cm below the marker in base, whatever way the camera is turned
}

TEST(TargetValues, AreTheRowMajorTransform) {
  const Eigen::Matrix3d orientation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto values = target_values(orientation, Eigen::Vector3d(0.4, -0.3, 1.1));
  EXPECT_NEAR(values[0], 0.0, 1e-12);
  EXPECT_NEAR(values[1], -1.0, 1e-12);  // row 0, column 1
  EXPECT_NEAR(values[4], 1.0, 1e-12);   // row 1, column 0
  EXPECT_NEAR(values[10], 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(values[3], 0.4);
  EXPECT_DOUBLE_EQ(values[7], -0.3);
  EXPECT_DOUBLE_EQ(values[11], 1.1);
  EXPECT_DOUBLE_EQ(values[12], 0.0);
  EXPECT_DOUBLE_EQ(values[13], 0.0);
  EXPECT_DOUBLE_EQ(values[14], 0.0);
  EXPECT_DOUBLE_EQ(values[15], 1.0);
}

TEST(NextMarker, GoesThroughTheMarkersInViewFromTheLowestIdUp) {
  using rby1_apriltag::next_marker;
  const std::vector<int64_t> seen{12, 7, 8};
  EXPECT_EQ(next_marker(seen, std::nullopt), 7);  // the lowest first
  EXPECT_EQ(next_marker(seen, 7), 8);
  EXPECT_EQ(next_marker(seen, 8), 12);
  EXPECT_EQ(next_marker(seen, 12), 7);  // round again
}

TEST(NextMarker, TakesWhatIsInViewNow) {
  using rby1_apriltag::next_marker;
  EXPECT_EQ(next_marker({}, std::nullopt), std::nullopt);
  EXPECT_EQ(next_marker({}, 7), std::nullopt);
  EXPECT_EQ(next_marker({8}, 8), 8);        // one marker: that one every time
  EXPECT_EQ(next_marker({7, 12}, 8), 12);   // 8 went out of view: on to the next above it
  EXPECT_EQ(next_marker({7, 8}, 12), 7);    // the last one sent is gone: from the lowest
}
