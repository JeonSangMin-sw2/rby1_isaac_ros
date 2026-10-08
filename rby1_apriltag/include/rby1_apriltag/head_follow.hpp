// Head tracking: when the head turns for a marker, and how far each tick. No ROS here;
// the node that uses it is in src/head_follow.cpp.
#pragma once

#include <array>
#include <optional>

#include <Eigen/Geometry>

namespace rby1_apriltag {

using HeadAngles = std::array<double, 2>;  // head_0 (pan, left is positive), head_1 (tilt, down is positive), rad

// The camera's optical frame (z forward, x right, y down) in the head link, from the roll,
// pitch, yaw of camera_link (x forward, y left, z up) there: Rz(yaw) Ry(pitch) Rx(roll).
Eigen::Matrix3d head_from_optical(const std::array<double, 3> & camera_rpy);

// How far `marker` (a position in the optical frame) is off the camera's line of sight,
// as the pan and tilt that bring it there. Zero for a marker on the line of sight.
HeadAngles look_error(const Eigen::Matrix3d & head_from_optical, const Eigen::Vector3d & marker);

// `command` moved toward `goal`, each angle by at most `step`.
HeadAngles step_toward(const HeadAngles & command, const HeadAngles & goal, double step);

struct FollowSettings {
  double safe_zone = 0.03;   // rad off the line of sight: inside it the head stands still
  double start_zone = 0.10;  // rad: beyond it the head follows at once
  double dwell = 1.0;        // s between the two zones before the head follows
  double speed = 0.5;        // rad/s of a head joint while following, with the marker far off
  // rad: with the marker's error inside it a joint turns slower, in proportion to the error.
  // The image is late, and a head that stops from full speed has turned on by then: at
  // 0.5 rad/s it went past the far edge of the safe zone and never came to rest.
  double slow_zone = 0.25;
  double rate = 20.0;        // Hz of the ticks: a joint turns speed / rate per tick
  HeadAngles lower{-1.5, -1.5}, upper{1.5, 1.5};  // rad, joint limits
};

// Decides from the marker's errors whether the head follows, and steps the command.
// A marker is inside a zone when its pan and its tilt error are both within it.
class HeadFollower {
public:
  // Throws std::invalid_argument, saying what to change, for settings that cannot work.
  explicit HeadFollower(const FollowSettings & settings);

  // The marker's error seen at `time` (s). Returns whether the head follows from now on:
  // it starts beyond the start zone, or after more than `dwell` outside the safe zone,
  // and stops once the marker is inside the safe zone.
  bool marker(const HeadAngles & error, double time);

  // The marker went out of sight: not following, and the time outside the safe zone starts anew.
  void lost();

  // `command` one tick on. While following, a joint whose own error is outside the safe
  // zone turns toward the marker, inside the limits: speed / rate, less inside the slow
  // zone; otherwise unchanged.
  HeadAngles step(const HeadAngles & command) const;

  // step() for a head that turns behind its commands and stands at `measured` now. A
  // command stays within the marker's error of the measured head, and a joint that stops
  // stepping stands where it is: left to reach commands that had run ahead, the head went
  // past the far edge of the safe zone, to and fro. (Stepping from the measured angle
  // instead crawled: the head covers only part of a small step in a tick.)
  HeadAngles follow(const HeadAngles & command, const HeadAngles & measured);

  bool following() const { return following_; }

private:
  FollowSettings settings_;
  HeadAngles error_{0.0, 0.0};
  std::array<bool, 2> stepping_{};  // follow() stepped that joint last time
  std::optional<double> left_safe_;  // when the marker left the safe zone
  bool following_ = false;
};

}  // namespace rby1_apriltag
