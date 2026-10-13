// marker_shuttle: the hand of one arm goes round the markers in view, lowest id first.
//
// Takes /rby1/marker_<id>/pose for every id in `marker_ids` (the launch gives it the
// ids target_tag_filter keeps), brings each into `frame` (base) through TF at the time
// of the image and moves it by `offset`. Every `period` seconds the next marker in view
// -- seen within `fresh` seconds -- goes out as a target, in the order of the ids:
//   /rby1/<arm>/target_pose   std_msgs/Float64MultiArray, 16 values: the row-major 4x4 of
//                             the arm's tool frame in `frame` -- what a target executor takes
// With markers 7, 8 and 12 in view: 7, 8, 12, 7, ... A marker taken away is left out, one
// that comes into view joins the round. The hand keeps the orientation it had when the
// first target went out. It only publishes: `period` must be longer than a move takes.
//
// Needs the robot's TF (a target executor's launch) and the camera mount's (camera.launch.py).
// With a target executor running, this moves the robot.
#include <chrono>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rby1_apriltag/hand_target.hpp"

using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Float64MultiArray;

namespace {

const auto kTfWait = std::chrono::milliseconds(100);  // for the robot's TF to reach the time of the image
const int kQuietMs = 5000;                            // between repeats of the same warning

Eigen::Isometry3d isometry(const geometry_msgs::msg::Transform & transform) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::Quaterniond(transform.rotation.w, transform.rotation.x, transform.rotation.y,
                                       transform.rotation.z).normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z);
  return result;
}

}  // namespace

class MarkerShuttle : public rclcpp::Node {
public:
  MarkerShuttle() : rclcpp::Node("marker_shuttle"), buffer_(get_clock()), listener_(buffer_) {
    frame_ = declare_parameter("frame", "base");
    const auto arm = declare_parameter("arm", "right_arm");
    if (arm != "right_arm" && arm != "left_arm") {
      throw std::invalid_argument("arm must be right_arm or left_arm, got '" + arm + "'");
    }
    tool_ = declare_parameter("tool_frame", arm == "left_arm" ? "ee_left" : "ee_right");
    topic_ = declare_parameter("target_topic", "/rby1/" + arm + "/target_pose");
    const auto offset = declare_parameter("offset", std::vector<double>{0.0, 0.0, -0.10});
    if (offset.size() != 3) throw std::invalid_argument("offset must be x, y, z in metres");
    offset_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);
    period_ = declare_parameter("period", 5.0);
    fresh_ = declare_parameter("fresh", 1.0);
    if (!(period_ > 0.0) || !(fresh_ > 0.0)) throw std::invalid_argument("period and fresh must be positive (s)");
    const auto ids = declare_parameter("marker_ids", std::vector<int64_t>{});
    if (ids.empty()) {
      throw std::invalid_argument("marker_ids is empty: marker_shuttle.launch.py gives it the ids the filter keeps");
    }
    const auto prefix = declare_parameter("marker_topic_prefix", "/rby1/marker");
    publisher_ = create_publisher<Float64MultiArray>(topic_, 10);
    std::ostringstream listed;
    for (const int64_t id : ids) {
      // Depth 1: a pose that waited behind a slow TF lookup is not worth acting on.
      subscriptions_.push_back(create_subscription<PoseStamped>(
        prefix + "_" + std::to_string(id) + "/pose", 1, [this, id](const PoseStamped & msg) { on_marker(id, msg); }));
      listed << (listed.tellp() > 0 ? ", " : "") << id;
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(period_), [this] { send_next(); });
    RCLCPP_INFO(get_logger(), "%s: every %.1f s to the next marker in view of [%s], + [%.3f, %.3f, %.3f] in %s -> %s",
                arm.c_str(), period_, listed.str().c_str(), offset[0], offset[1], offset[2], frame_.c_str(),
                topic_.c_str());
  }

private:
  struct Seen {
    Eigen::Vector3d point;  // where the hand goes for it, in frame_
    rclcpp::Time when;
  };

  void on_marker(int64_t id, const PoseStamped & msg) {
    try {
      const Eigen::Isometry3d frame_from_camera = isometry(
        buffer_.lookupTransform(frame_, msg.header.frame_id, rclcpp::Time(msg.header.stamp),
                                rclcpp::Duration(kTfWait)).transform);
      seen_[id] = Seen{rby1_apriltag::hand_point(
                         frame_from_camera,
                         Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), offset_),
                       now()};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kQuietMs,
                           "marker %ld is seen but its place in %s is unknown -- the robot's TF comes from a "
                           "target executor's launch, the camera's from camera.launch.py (%s)",
                           id, frame_.c_str(), error.what());
    }
  }

  void send_next() {
    std::vector<int64_t> in_view;
    std::ostringstream listed;
    for (const auto & [id, seen] : seen_) {
      if ((now() - seen.when).seconds() > fresh_) continue;
      in_view.push_back(id);
      listed << (listed.tellp() > 0 ? ", " : "") << id;
    }
    const auto next = rby1_apriltag::next_marker(in_view, last_);
    if (!next) {
      RCLCPP_INFO(get_logger(), "no marker in view: are they in the image, and are the camera and the detector up?");
      return;
    }
    if (publisher_->get_subscription_count() == 0) {
      RCLCPP_INFO(get_logger(), "no target executor listens on %s", topic_.c_str());
      return;
    }
    if (!orientation_) {
      try {
        orientation_ = isometry(buffer_.lookupTransform(frame_, tool_, tf2::TimePointZero).transform).rotation();
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN(get_logger(), "where %s is in %s is unknown (%s)", tool_.c_str(), frame_.c_str(), error.what());
        return;
      }
    }
    const Eigen::Vector3d & point = seen_.at(*next).point;
    Float64MultiArray target;
    const auto values = rby1_apriltag::target_values(*orientation_, point);
    target.data.assign(values.begin(), values.end());
    publisher_->publish(target);
    last_ = next;
    RCLCPP_INFO(get_logger(), "-> marker %ld: target [%.3f, %.3f, %.3f] in %s (in view: %s)", *next, point.x(),
                point.y(), point.z(), frame_.c_str(), listed.str().c_str());
  }

  std::string frame_, tool_, topic_;
  Eigen::Vector3d offset_;
  double period_, fresh_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::map<int64_t, Seen> seen_;
  std::optional<int64_t> last_;                // the marker the last target was for
  std::optional<Eigen::Matrix3d> orientation_;  // the hand's, kept from the first target on
  rclcpp::Publisher<Float64MultiArray>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<PoseStamped>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MarkerShuttle>());
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(rclcpp::get_logger("marker_shuttle"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
