// head_follow: the head turns after a marker that leaves the middle of the camera image.
//
// Started by apriltag.launch.py when head following is on (follow_head:=true, or
// head_follow: enabled in config/target_tags.yaml). Takes /rby1/marker_<id>/pose (the
// marker in the camera's optical frame, from target_tag_filter) and works out how far
// the marker is off the camera's line of sight, with the camera's mounting rotation
// from `camera_rpy` (no TF is read).
//
// Inside `safe_zone` the head stands still. Beyond `start_zone`, or after `dwell`
// between the two, it follows: each tick the head joints turn speed / rate toward the
// marker, until the marker is inside the safe zone again. Commands are the driver's
// stream_joint, head joints only, on the driver's 'head' stream channel -- so an arm
// trajectory or an arm stream runs alongside.
//
// Marker out of sight for `lost_timeout`: the head holds. For `return_after`: it goes
// to `home` and waits there. A command the driver refuses (its channel was closed):
// the channel is asked for again. Stopped (Ctrl+C), it closes the channel it opened:
// an open channel makes the driver refuse other commands for parts whose channel is not.
//
// Needs the driver (robot powered, head servo on). Moves the robot's head.
#include "rby1_apriltag/head_follow.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rby1_apriltag {

namespace {

// Pan and tilt of a direction given along the head link's axes (x ahead, y left, z up).
HeadAngles pan_tilt(const Eigen::Vector3d & direction) {
  return {std::atan2(direction.y(), direction.x()),
          -std::atan2(direction.z(), std::hypot(direction.x(), direction.y()))};
}

void need_positive(double value, const char * name, const char * unit) {
  if (value > 0.0) return;
  std::ostringstream text;
  text << name << " must be positive (" << unit << "), got " << value << ": change head_follow: " << name
       << " in the settings file (target_tags.yaml)";
  throw std::invalid_argument(text.str());
}

const double kLead = 0.5;  // s at `speed`: how far a command may run ahead of the measured head

}  // namespace

Eigen::Matrix3d head_from_optical(const std::array<double, 3> & camera_rpy) {
  const Eigen::Matrix3d head_from_camera = (Eigen::AngleAxisd(camera_rpy[2], Eigen::Vector3d::UnitZ()) *
                                            Eigen::AngleAxisd(camera_rpy[1], Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(camera_rpy[0], Eigen::Vector3d::UnitX()))
                                             .toRotationMatrix();
  Eigen::Matrix3d camera_from_optical;  // optical x is the camera's -y, y its -z, z its x
  camera_from_optical << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  return head_from_camera * camera_from_optical;
}

HeadAngles look_error(const Eigen::Matrix3d & head_from_optical, const Eigen::Vector3d & marker) {
  const HeadAngles to_marker = pan_tilt(head_from_optical * marker);
  const HeadAngles sight = pan_tilt(head_from_optical * Eigen::Vector3d::UnitZ());
  return {std::remainder(to_marker[0] - sight[0], 2.0 * M_PI), to_marker[1] - sight[1]};
}

HeadAngles step_toward(const HeadAngles & command, const HeadAngles & goal, double step) {
  HeadAngles result = command;
  for (size_t i = 0; i < result.size(); ++i) result[i] += std::clamp(goal[i] - command[i], -step, step);
  return result;
}

HeadFollower::HeadFollower(const FollowSettings & settings) : settings_(settings) {
  need_positive(settings.safe_zone, "safe_zone", "rad");
  need_positive(settings.start_zone, "start_zone", "rad");
  need_positive(settings.dwell, "dwell", "s");
  need_positive(settings.speed, "speed", "rad/s");
  need_positive(settings.slow_zone, "slow_zone", "rad");
  need_positive(settings.rate, "rate", "Hz");
  if (settings.start_zone <= settings.safe_zone) {
    std::ostringstream text;
    text << "start_zone (" << settings.start_zone << " rad) must be larger than safe_zone (" << settings.safe_zone
         << " rad): raise head_follow: start_zone or lower safe_zone in the settings file (target_tags.yaml)";
    throw std::invalid_argument(text.str());
  }
  for (size_t i = 0; i < settings.lower.size(); ++i) {
    if (settings.lower[i] < settings.upper[i]) continue;
    std::ostringstream text;
    text << "lower_limits[" << i << "] (" << settings.lower[i] << " rad) must be below upper_limits[" << i << "] ("
         << settings.upper[i] << " rad): change head_follow: lower_limits or upper_limits";
    throw std::invalid_argument(text.str());
  }
}

bool HeadFollower::marker(const HeadAngles & error, double time) {
  error_ = error;
  const double off = std::max(std::abs(error[0]), std::abs(error[1]));
  if (off <= settings_.safe_zone) {
    following_ = false;
    left_safe_.reset();
    return false;
  }
  if (!left_safe_) left_safe_ = time;
  if (off > settings_.start_zone || time - *left_safe_ > settings_.dwell) following_ = true;
  return following_;
}

void HeadFollower::lost() {
  following_ = false;
  left_safe_.reset();
}

HeadAngles HeadFollower::step(const HeadAngles & command) const {
  HeadAngles result = command;
  if (!following_) return result;
  for (size_t i = 0; i < result.size(); ++i) {
    if (std::abs(error_[i]) <= settings_.safe_zone) continue;
    const double step = settings_.speed / settings_.rate * std::min(1.0, std::abs(error_[i]) / settings_.slow_zone);
    const double wanted =
      std::clamp(result[i] + std::copysign(step, error_[i]), settings_.lower[i], settings_.upper[i]);
    // A joint that starts outside its limits comes back a step at a time, not in one jump.
    result[i] += std::clamp(wanted - result[i], -step, step);
  }
  return result;
}

HeadAngles HeadFollower::follow(const HeadAngles & command, const HeadAngles & measured) {
  HeadAngles next = step(command);
  for (size_t i = 0; i < next.size(); ++i) {
    const bool steps = next[i] != command[i];
    if (steps) {
      const double lead = std::min(settings_.speed * kLead, std::abs(error_[i]));
      next[i] = std::clamp(next[i], measured[i] - lead, measured[i] + lead);
    } else if (stepping_[i]) {
      next[i] = measured[i];
    }
    stepping_[i] = steps;
  }
  return next;
}

}  // namespace rby1_apriltag

// The node and main. The unit test builds this file with HEAD_FOLLOW_NO_MAIN, so that
// it links the logic above without ROS and without a second main.
#ifndef HEAD_FOLLOW_NO_MAIN

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rby1_msgs/action/stream_joint.hpp>
#include <rby1_msgs/srv/state_on_off.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using geometry_msgs::msg::PoseStamped;
using rby1_apriltag::HeadAngles;
using rby1_msgs::action::StreamJoint;
using rby1_msgs::srv::StateOnOff;

namespace {

const std::vector<std::string> kHeadJoints{"head_0", "head_1"};
const char kChannel[] = "head";  // the driver's stream channel (stream_control's parameters)
const char kOpened[] = "Stream channels opened";  // how the driver's answer starts when it opened one
std::atomic<bool> stop_asked{false};              // SIGINT / SIGTERM: main closes the channel, then ends
const double kRetryAfter = 0.5;  // s, after the driver refused a command or the channel
const int kQuietMs = 5000;       // between repeats of the same warning

HeadAngles pair(const std::vector<double> & values, const std::string & name) {
  if (values.size() != 2) throw std::invalid_argument(name + " needs 2 values: head_0, head_1 (rad)");
  return {values[0], values[1]};
}

}  // namespace

class HeadFollow : public rclcpp::Node {
public:
  HeadFollow() : rclcpp::Node("head_follow") {
    const auto marker = declare_parameter<int64_t>("marker_id", 7);
    const auto prefix = declare_parameter("marker_topic_prefix", "/rby1/marker");
    auto driver = declare_parameter("driver_namespace", "rby1");
    while (!driver.empty() && driver.front() == '/') driver.erase(driver.begin());
    while (!driver.empty() && driver.back() == '/') driver.pop_back();
    const std::string root = driver.empty() ? "" : "/" + driver;
    settings_.safe_zone = declare_parameter("safe_zone", settings_.safe_zone);
    settings_.start_zone = declare_parameter("start_zone", settings_.start_zone);
    settings_.dwell = declare_parameter("dwell", settings_.dwell);
    settings_.speed = declare_parameter("speed", settings_.speed);
    settings_.slow_zone = declare_parameter("slow_zone", settings_.slow_zone);
    settings_.rate = declare_parameter("rate", settings_.rate);
    const auto rpy = declare_parameter("camera_rpy", std::vector<double>{0.0, 0.0, 0.0});
    lost_timeout_ = declare_parameter("lost_timeout", 0.5);
    return_after_ = declare_parameter("return_after", 3.0);
    home_ = pair(declare_parameter("home", std::vector<double>{0.0, 0.0}), "home");
    settings_.lower = pair(declare_parameter("lower_limits", std::vector<double>{-1.5, -1.5}), "lower_limits");
    settings_.upper = pair(declare_parameter("upper_limits", std::vector<double>{1.5, 1.5}), "upper_limits");
    if (rpy.size() != 3) {
      throw std::invalid_argument("camera_rpy needs 3 values: roll, pitch, yaw of camera_link in link_head_2 (rad), "
                                  "as rpy in rby1_additional_tools' camera_mount.yaml");
    }
    head_from_optical_ = rby1_apriltag::head_from_optical({rpy[0], rpy[1], rpy[2]});
    follower_.emplace(settings_);  // refuses settings that cannot work

    const auto topic = prefix + "_" + std::to_string(marker) + "/pose";
    stream_ = rclcpp_action::create_client<StreamJoint>(this, root + "/stream_joint");
    channel_ = create_client<StateOnOff>(root + "/stream_control");
    marker_sub_ = create_subscription<PoseStamped>(topic, 10, [this](const PoseStamped & msg) { on_marker(msg); });
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      root + "/joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState & msg) {
        HeadAngles angles{};
        size_t found = 0;
        for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
          for (size_t k = 0; k < kHeadJoints.size(); ++k) {
            if (msg.name[i] == kHeadJoints[k]) {
              angles[k] = msg.position[i];
              ++found;
            }
          }
        }
        if (found == kHeadJoints.size()) measured_ = angles;
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / settings_.rate), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "the head follows %s at %.0f Hz: still within %.3f rad of the camera's line of sight, "
                "following beyond %.3f rad or after %.1f s in between, at %.2f rad/s (camera rpy [%.3f, %.3f, %.3f] "
                "in link_head_2)", topic.c_str(), settings_.rate, settings_.safe_zone, settings_.start_zone,
                settings_.dwell, settings_.speed, rpy[0], rpy[1], rpy[2]);
  }

  // Closes the head's channel if this node opened it; one that was open before is left.
  void close_channel() {
    timer_->cancel();
    if (!channel_mine_ || !channel_->service_is_ready()) return;
    auto request = std::make_shared<StateOnOff::Request>();
    request->state = false;
    request->parameters = kChannel;
    auto future = channel_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, std::chrono::seconds(2)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "the driver did not answer: the head's stream channel may still be open");
    }
  }

private:
  enum class Mode { kIdle, kTracking, kHolding, kReturning };

  double seconds() { return now().seconds(); }

  void on_marker(const PoseStamped & msg) {
    if (!measured_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kQuietMs,
                           "marker seen but no head angles yet: is the RB-Y1 driver publishing joint_states?");
      return;
    }
    // Only the mount's rotation: the direction from the camera, not from the head joints
    // (the camera sits off them, so aiming the head link would leave a close marker off-centre).
    const Eigen::Vector3d marker(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    const HeadAngles error = rby1_apriltag::look_error(head_from_optical_, marker);
    last_marker_ = seconds();
    if (mode_ != Mode::kTracking) {
      RCLCPP_INFO(get_logger(), "marker found %.2f m away, [%.3f, %.3f] rad off the line of sight", marker.norm(),
                  error[0], error[1]);
      mode_ = Mode::kTracking;
    }
    const bool was_following = follower_->following();
    const bool following = follower_->marker(error, last_marker_);
    if (following && !was_following) {
      const bool beyond = std::max(std::abs(error[0]), std::abs(error[1])) > settings_.start_zone;
      RCLCPP_INFO(get_logger(), "following: the marker is [%.3f, %.3f] rad off the line of sight (%s)", error[0],
                  error[1], beyond ? "beyond the start zone" : "outside the safe zone for longer than dwell");
    } else if (!following && was_following) {
      RCLCPP_INFO(get_logger(), "marker back inside the safe zone ([%.3f, %.3f] rad off): the head stands still",
                  error[0], error[1]);
    }
  }

  void tick() {
    if (last_marker_ < 0.0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kQuietMs,
                           "no marker yet: is it in view, and are the camera and the detector up?");
    }
    if (mode_ == Mode::kIdle || seconds() < paused_until_) return;
    const double lost = seconds() - last_marker_;
    if (mode_ == Mode::kTracking && lost > lost_timeout_) {
      RCLCPP_INFO(get_logger(), "marker lost: holding the head (home after %.1f s without it)", return_after_);
      mode_ = Mode::kHolding;
      follower_->lost();
    }
    if (mode_ == Mode::kHolding && return_after_ > 0.0 && lost > return_after_) {
      RCLCPP_INFO(get_logger(), "going home [%.2f, %.2f]", home_[0], home_[1]);
      mode_ = Mode::kReturning;
    }
    if (!command_) command_ = measured_;  // start from where the head is
    if (!command_) return;
    if (mode_ == Mode::kReturning && std::abs((*command_)[0] - home_[0]) < 1e-3 &&
        std::abs((*command_)[1] - home_[1]) < 1e-3) {
      RCLCPP_INFO(get_logger(), "home: waiting for the marker");
      mode_ = Mode::kIdle;
      command_.reset();
      return;
    }
    if (!channel_open_) {
      open_channel();
      return;
    }
    // Standing still or holding, the same command goes out again: it keeps the channel open.
    if ((mode_ == Mode::kTracking || mode_ == Mode::kHolding) && measured_) {
      command_ = follower_->follow(*command_, *measured_);
    }
    if (mode_ == Mode::kReturning) {
      command_ = rby1_apriltag::step_toward(*command_, home_, settings_.speed / settings_.rate);
    }
    send(*command_);
  }

  void open_channel() {
    if (channel_asked_) return;
    if (!channel_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kQuietMs, "%s is not available: is the RB-Y1 driver running?",
                           channel_->get_service_name());
      return;
    }
    auto request = std::make_shared<StateOnOff::Request>();
    request->state = true;
    request->parameters = kChannel;
    channel_asked_ = true;
    channel_->async_send_request(request, [this](rclcpp::Client<StateOnOff>::SharedFuture future) {
      channel_asked_ = false;
      const auto response = future.get();
      channel_open_ = response->success;
      if (response->success && response->message.rfind(kOpened, 0) == 0) channel_mine_ = true;
      if (response->success) {
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kQuietMs, "the driver did not open the head's stream "
                             "channel: %s", response->message.c_str());
        paused_until_ = seconds() + kRetryAfter;
      }
    });
  }

  void send(const HeadAngles & head) {
    if (!stream_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kQuietMs,
                           "stream_joint is not available: is the RB-Y1 driver running?");
      return;
    }
    StreamJoint::Goal goal;
    goal.command.head.joint_names = kHeadJoints;
    goal.command.head.position = {head[0], head[1]};
    goal.command.head.minimum_time = 1.0 / settings_.rate;
    rclcpp_action::Client<StreamJoint>::SendGoalOptions options;
    options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<StreamJoint>::SharedPtr handle) {
      if (!handle) refused("the driver rejected the goal (hardware control active?)");
    };
    options.result_callback = [this](const rclcpp_action::ClientGoalHandle<StreamJoint>::WrappedResult & result) {
      if (!result.result || !result.result->success) {
        refused("the driver dropped it (a trajectory holds the head, or the head's stream channel closed) -- "
                "see the driver log");
      }
    };
    stream_->async_send_goal(goal, options);
  }

  // Start again from where the head is, on a channel asked for anew: the driver closes
  // every channel after 60 s without a command, and anyone may close this one.
  void refused(const char * reason) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "command refused: %s; retrying in %.1f s", reason,
                         kRetryAfter);
    paused_until_ = seconds() + kRetryAfter;
    command_.reset();
    channel_open_ = false;
  }

  rby1_apriltag::FollowSettings settings_;
  std::optional<rby1_apriltag::HeadFollower> follower_;
  Eigen::Matrix3d head_from_optical_;
  double lost_timeout_, return_after_;
  HeadAngles home_;
  Mode mode_ = Mode::kIdle;
  std::optional<HeadAngles> measured_, command_;
  double last_marker_ = -1e18, paused_until_ = -1e18;
  bool channel_open_ = false, channel_asked_ = false, channel_mine_ = false;
  rclcpp_action::Client<StreamJoint>::SharedPtr stream_;
  rclcpp::Client<StateOnOff>::SharedPtr channel_;
  rclcpp::Subscription<PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  // Signals are handled here, not by rclcpp: the channel is closed with a service call,
  // which needs the context still up.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { stop_asked = true; });
  std::signal(SIGTERM, [](int) { stop_asked = true; });
  try {
    auto node = std::make_shared<HeadFollow>();
    while (rclcpp::ok() && !stop_asked) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    node->close_channel();
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(rclcpp::get_logger("head_follow"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#endif  // HEAD_FOLLOW_NO_MAIN
