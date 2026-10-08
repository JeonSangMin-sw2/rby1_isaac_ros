// When the head follows a marker and how far it steps; no ROS graph.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "rby1_apriltag/head_follow.hpp"

using rby1_apriltag::FollowSettings;
using rby1_apriltag::HeadAngles;
using rby1_apriltag::HeadFollower;
using rby1_apriltag::head_from_optical;
using rby1_apriltag::look_error;
using rby1_apriltag::step_toward;

namespace {

const double kStep = 0.5 / 20.0;  // the default speed / rate

// A point in the optical frame (x right, y down, z forward) `left` and `up` rad off its z axis.
Eigen::Vector3d seen(double left, double up) {
  return 0.7 * Eigen::Vector3d(-std::sin(left) * std::cos(up), -std::sin(up), std::cos(left) * std::cos(up));
}

}  // namespace

TEST(HeadFromOptical, TheDefaultMountIsTheFixedConversion) {
  const Eigen::Matrix3d rotation = head_from_optical({0.0, 0.0, 0.0});
  EXPECT_TRUE((rotation * Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitX(), 1e-12));   // forward
  EXPECT_TRUE((rotation * Eigen::Vector3d::UnitX()).isApprox(-Eigen::Vector3d::UnitY(), 1e-12));  // right
  EXPECT_TRUE((rotation * Eigen::Vector3d::UnitY()).isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));  // down
}

TEST(LookError, DefaultMountAheadLeftAndAbove) {
  const Eigen::Matrix3d mount = head_from_optical({0.0, 0.0, 0.0});
  const auto ahead = look_error(mount, seen(0.0, 0.0));
  EXPECT_NEAR(ahead[0], 0.0, 1e-12);
  EXPECT_NEAR(ahead[1], 0.0, 1e-12);
  const auto left = look_error(mount, seen(0.3, 0.0));
  EXPECT_NEAR(left[0], 0.3, 1e-12);  // left is positive pan
  EXPECT_NEAR(left[1], 0.0, 1e-12);
  const auto above = look_error(mount, seen(0.0, 0.2));
  EXPECT_NEAR(above[0], 0.0, 1e-12);
  EXPECT_NEAR(above[1], -0.2, 1e-12);  // up is negative tilt
}

TEST(LookError, MountPitchedDownAheadLeftAndAbove) {
  const double pitch = 0.3;  // camera_link nose down
  const Eigen::Matrix3d mount = head_from_optical({0.0, pitch, 0.0});
  const auto ahead = look_error(mount, seen(0.0, 0.0));
  EXPECT_NEAR(ahead[0], 0.0, 1e-12);  // on the line of sight, wherever the camera points
  EXPECT_NEAR(ahead[1], 0.0, 1e-12);
  const auto above = look_error(mount, seen(0.0, 0.2));
  EXPECT_NEAR(above[0], 0.0, 1e-12);
  EXPECT_NEAR(above[1], -0.2, 1e-12);
  const auto left = look_error(mount, seen(0.1, 0.0));
  EXPECT_NEAR(left[0], std::atan2(std::sin(0.1), std::cos(0.1) * std::cos(pitch)), 1e-12);
  EXPECT_GT(left[0], 0.1);  // about the head's upright axis the same marker is further round
}

TEST(LookError, MountYawedAndRolled) {
  // The camera looks 0.2 rad to the left: a marker straight ahead of the head is to its right.
  const auto ahead_of_head = look_error(head_from_optical({0.0, 0.0, 0.2}), seen(-0.2, 0.0));
  EXPECT_NEAR(ahead_of_head[0], -0.2, 1e-12);
  EXPECT_NEAR(ahead_of_head[1], 0.0, 1e-12);
  // Rolled a quarter turn to its right, the image's left is the head's up.
  const auto rolled = look_error(head_from_optical({M_PI / 2, 0.0, 0.0}), seen(0.2, 0.0));
  EXPECT_NEAR(rolled[0], 0.0, 1e-12);
  EXPECT_NEAR(rolled[1], -0.2, 1e-12);
}

TEST(StepToward, GoesAtMostOneStepAndNotPastTheGoal) {
  const HeadAngles stepped = step_toward({0.0, 0.0}, {1.0, -0.01}, 0.04);
  EXPECT_NEAR(stepped[0], 0.04, 1e-12);
  EXPECT_NEAR(stepped[1], -0.01, 1e-12);
}

TEST(HeadFollower, StandsStillInsideTheSafeZone) {
  HeadFollower follower{FollowSettings{}};
  for (double time = 0.0; time < 5.0; time += 0.05) EXPECT_FALSE(follower.marker({0.03, -0.02}, time));
  const HeadAngles stepped = follower.step({0.2, -0.1});
  EXPECT_DOUBLE_EQ(stepped[0], 0.2);
  EXPECT_DOUBLE_EQ(stepped[1], -0.1);
}

TEST(HeadFollower, StartsAtOnceBeyondTheStartZone) {
  HeadFollower pan{FollowSettings{}};
  EXPECT_TRUE(pan.marker({0.11, 0.0}, 0.0));
  HeadFollower tilt{FollowSettings{}};
  EXPECT_TRUE(tilt.marker({0.0, -0.11}, 0.0));
}

TEST(HeadFollower, BetweenTheZonesStartsAfterDwellAndNotBefore) {
  HeadFollower follower{FollowSettings{}};
  EXPECT_FALSE(follower.marker({0.06, 0.0}, 10.0));
  EXPECT_FALSE(follower.marker({0.06, 0.0}, 10.9));
  EXPECT_FALSE(follower.marker({0.06, 0.0}, 11.0));  // dwell, not yet longer
  EXPECT_NEAR(follower.step({0.0, 0.0})[0], 0.0, 1e-12);
  EXPECT_TRUE(follower.marker({0.06, 0.0}, 11.05));
}

TEST(HeadFollower, ComingBackIntoTheSafeZoneStartsTheDwellAnew) {
  HeadFollower follower{FollowSettings{}};
  EXPECT_FALSE(follower.marker({0.06, 0.0}, 0.0));
  EXPECT_FALSE(follower.marker({0.01, 0.0}, 0.8));
  EXPECT_FALSE(follower.marker({0.06, 0.0}, 1.2));
  EXPECT_FALSE(follower.marker({0.06, 0.0}, 2.1));
  EXPECT_TRUE(follower.marker({0.06, 0.0}, 2.3));
}

TEST(HeadFollower, LosingTheMarkerStopsFollowingAndStartsTheDwellAnew) {
  HeadFollower follower{FollowSettings{}};
  EXPECT_TRUE(follower.marker({0.2, 0.0}, 0.0));
  follower.lost();
  EXPECT_FALSE(follower.following());
  EXPECT_NEAR(follower.step({0.0, 0.0})[0], 0.0, 1e-12);
  EXPECT_FALSE(follower.marker({0.06, 0.0}, 5.0));
  EXPECT_TRUE(follower.marker({0.06, 0.0}, 6.1));
}

TEST(HeadFollower, StepsBySpeedOverRateTowardTheMarkerAndStopsInsideTheSafeZone) {
  HeadFollower follower{FollowSettings{}};
  ASSERT_TRUE(follower.marker({0.3, -0.3}, 0.0));
  HeadAngles command{0.1, 0.1};
  command = follower.step(command);
  EXPECT_NEAR(command[0], 0.1 + kStep, 1e-12);
  EXPECT_NEAR(command[1], 0.1 - kStep, 1e-12);
  command = follower.step(command);  // no new marker pose: on in the same direction
  EXPECT_NEAR(command[0], 0.1 + 2 * kStep, 1e-12);
  EXPECT_NEAR(command[1], 0.1 - 2 * kStep, 1e-12);
  // Between the zones it keeps following: it stops in the safe zone, not at the start zone.
  // Inside the slow zone (0.25 rad) the step is the share of it the marker is still off.
  EXPECT_TRUE(follower.marker({0.06, -0.06}, 0.1));
  command = follower.step(command);
  EXPECT_NEAR(command[0], 0.1 + 2 * kStep + kStep * 0.06 / 0.25, 1e-12);
  EXPECT_NEAR(command[1], 0.1 - 2 * kStep - kStep * 0.06 / 0.25, 1e-12);
  EXPECT_FALSE(follower.marker({0.02, -0.02}, 0.2));
  const HeadAngles stopped = follower.step(command);
  EXPECT_DOUBLE_EQ(stopped[0], command[0]);
  EXPECT_DOUBLE_EQ(stopped[1], command[1]);
}

TEST(HeadFollower, OnlyTheJointThatIsOffMoves) {
  HeadFollower follower{FollowSettings{}};
  ASSERT_TRUE(follower.marker({-0.2, 0.01}, 0.0));
  const HeadAngles command = follower.step({0.0, 0.4});
  EXPECT_NEAR(command[0], -kStep * 0.2 / 0.25, 1e-12);
  EXPECT_DOUBLE_EQ(command[1], 0.4);
}

TEST(HeadFollower, ACommandStaysWithinTheMarkersErrorOfTheMeasuredHead) {
  HeadFollower follower{FollowSettings{}};
  ASSERT_TRUE(follower.marker({0.2, -0.05}, 0.0));
  HeadAngles command{0.0, 0.0};
  for (int i = 0; i < 40; ++i) command = follower.follow(command, {0.0, 0.0});  // a head that does not turn
  EXPECT_NEAR(command[0], 0.2, 1e-12);
  EXPECT_NEAR(command[1], -0.05, 1e-12);
  // Far off, not further than the head covers in half a second at `speed`.
  ASSERT_TRUE(follower.marker({1.0, 0.0}, 0.1));
  for (int i = 0; i < 40; ++i) command = follower.follow(command, {0.0, 0.0});
  EXPECT_NEAR(command[0], 0.25, 1e-12);
}

TEST(HeadFollower, AJointThatStopsSteppingStandsWhereTheHeadIs) {
  HeadFollower follower{FollowSettings{}};
  ASSERT_TRUE(follower.marker({0.2, 0.2}, 0.0));
  HeadAngles command = follower.follow({0.0, 0.0}, {0.0, 0.0});
  EXPECT_NEAR(command[0], kStep * 0.2 / 0.25, 1e-12);
  // Tilt is inside the safe zone now, pan still off: tilt stands at the measured angle.
  ASSERT_TRUE(follower.marker({0.2, 0.01}, 0.1));
  command = follower.follow(command, {0.01, 0.015});
  EXPECT_GT(command[0], 0.01);
  EXPECT_DOUBLE_EQ(command[1], 0.015);
  // And stays there: it is not pulled along with the measured angle afterwards.
  command = follower.follow(command, {0.02, 0.011});
  EXPECT_DOUBLE_EQ(command[1], 0.015);
  // The marker inside the safe zone: pan stands where the head is, too.
  ASSERT_FALSE(follower.marker({0.02, 0.01}, 0.2));
  command = follower.follow(command, {0.03, 0.011});
  EXPECT_DOUBLE_EQ(command[0], 0.03);
  EXPECT_DOUBLE_EQ(command[1], 0.015);
}

// A head that reaches a command only after a while (0.4 s, as the simulator's does) and
// an image that is two ticks old: the head comes to rest in the safe zone the first time.
TEST(HeadFollower, ASlowHeadComesToRestInsideTheSafeZoneWithoutGoingBack) {
  const FollowSettings settings;
  HeadFollower follower{settings};
  const HeadAngles marker{-0.36, 0.34};
  const double tick = 1.0 / settings.rate, lag = 0.4;
  HeadAngles head{0.0, 0.0}, command = head, before_last = head, last = head;
  int starts = 0;
  for (int i = 0; i < 400; ++i) {
    const bool was_following = follower.following();
    const HeadAngles seen_error{marker[0] - before_last[0], marker[1] - before_last[1]};
    if (follower.marker(seen_error, i * tick) && !was_following) ++starts;
    command = follower.follow(command, head);
    before_last = last;
    last = head;
    for (size_t k = 0; k < head.size(); ++k) {
      head[k] += (command[k] - head[k]) * tick / lag;
      EXPECT_LT(std::abs(head[k]), std::abs(marker[k]) + settings.safe_zone) << "tick " << i;  // not past the far edge
    }
  }
  EXPECT_EQ(starts, 1);
  EXPECT_FALSE(follower.following());
  for (size_t k = 0; k < head.size(); ++k) EXPECT_LE(std::abs(marker[k] - head[k]), settings.safe_zone);
}

TEST(HeadFollower, StaysInsideTheLimits) {
  FollowSettings settings;
  settings.lower = {-1.0, -0.5};
  settings.upper = {1.0, 0.5};
  HeadFollower follower{settings};
  ASSERT_TRUE(follower.marker({0.5, -0.5}, 0.0));
  HeadAngles command{0.99, -0.49};
  for (int i = 0; i < 10; ++i) command = follower.step(command);
  EXPECT_DOUBLE_EQ(command[0], 1.0);
  EXPECT_DOUBLE_EQ(command[1], -0.5);
  // From outside the limits it comes back one step a tick.
  ASSERT_TRUE(follower.marker({-0.5, 0.0}, 0.1));
  EXPECT_NEAR(follower.step({1.2, 0.0})[0], 1.2 - kStep, 1e-12);
  ASSERT_TRUE(follower.marker({0.5, 0.0}, 0.2));
  EXPECT_NEAR(follower.step({1.2, 0.0})[0], 1.2 - kStep, 1e-12);
}

TEST(HeadFollower, RefusesSettingsThatCannotWork) {
  FollowSettings settings;
  settings.start_zone = settings.safe_zone;
  EXPECT_THROW(HeadFollower{settings}, std::invalid_argument);
  settings.start_zone = 0.5 * settings.safe_zone;
  EXPECT_THROW(HeadFollower{settings}, std::invalid_argument);
  for (double FollowSettings::* value : {&FollowSettings::safe_zone, &FollowSettings::start_zone,
                                          &FollowSettings::dwell, &FollowSettings::speed, &FollowSettings::rate}) {
    for (const double bad : {0.0, -1.0}) {
      FollowSettings wrong;
      wrong.*value = bad;
      EXPECT_THROW(HeadFollower{wrong}, std::invalid_argument);
    }
  }
  FollowSettings limits;
  limits.lower = {0.5, -1.5};
  limits.upper = {0.5, 1.5};
  EXPECT_THROW(HeadFollower{limits}, std::invalid_argument);
  EXPECT_NO_THROW(HeadFollower{FollowSettings{}});
}
