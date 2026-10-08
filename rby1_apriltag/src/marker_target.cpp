// marker_target: a marker's pose from target_tag_filter, as a target for an arm.
//
// For each arm that has a marker (`<arm>.marker_id`): takes /rby1/marker_<id>/pose
// (the marker in the camera's optical frame), brings it into `frame` (base) through TF
// at the time of the image, moves it by `<arm>.offset` along `frame`'s axes, and publishes
//   /rby1/<arm>/target_pose   std_msgs/Float64MultiArray, 16 values: the row-major 4x4 of
//                             the arm's tool frame in `frame` -- what a target executor takes
// Only the hand's position follows the marker: it keeps the orientation it had when its
// first target went out. A new target goes out when it is `min_move` or more from the
// last one sent, and only while an executor listens, so a marker at rest gives one move,
// not one per image.
//
// Needs the robot's TF (a target executor's launch) and the camera mount's (camera.launch.py).
// With a target executor running, this moves the robot.
#include <chrono>
#include <memory>
#include <optional>
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

class MarkerTarget : public rclcpp::Node {
public:
  MarkerTarget() : rclcpp::Node("marker_target"), buffer_(get_clock()), listener_(buffer_) {
    frame_ = declare_parameter("frame", "base");
    min_move_ = declare_parameter("min_move", 0.01);
    if (min_move_ < 0.0) throw std::invalid_argument("min_move is a distance in metres, got a negative one");
    const auto prefix = declare_parameter("marker_topic_prefix", "/rby1/marker");
    add_arm("right_arm", 7, "ee_right", prefix);
    add_arm("left_arm", -1, "ee_left", prefix);
    if (arms_.empty()) throw std::invalid_argument("no arm has a marker: set right_arm.marker_id or left_arm.marker_id");
    waiting_ = create_wall_timer(std::chrono::milliseconds(kQuietMs), [this] {
      for (const auto & arm : arms_) {
        if (!arm->seen) {
          RCLCPP_INFO(get_logger(), "%s: no marker %ld yet: is it in view, and are the camera and the detector up?",
                      arm->name.c_str(), arm->marker);
        }
      }
    });
  }

private:
  struct Arm {
    std::string name, tool, topic;
    int64_t marker;
    Eigen::Vector3d offset;
    bool seen = false;                           // its marker came at least once
    std::optional<Eigen::Matrix3d> orientation;  // the hand's, kept from its first target on
    std::optional<Eigen::Vector3d> sent;         // the last target's position
    rclcpp::Publisher<Float64MultiArray>::SharedPtr publisher;
    rclcpp::Subscription<PoseStamped>::SharedPtr subscription;
  };

  void add_arm(const std::string & name, int64_t default_marker, const std::string & default_tool,
               const std::string & prefix) {
    auto arm = std::make_shared<Arm>();
    arm->name = name;
    arm->marker = declare_parameter(name + ".marker_id", default_marker);
    const auto offset = declare_parameter(name + ".offset", std::vector<double>{0.0, 0.0, -0.10});
    arm->tool = declare_parameter(name + ".tool_frame", default_tool);
    arm->topic = declare_parameter(name + ".target_topic", "/rby1/" + name + "/target_pose");
    if (arm->marker < 0) return;  // this arm follows no marker
    if (offset.size() != 3) throw std::invalid_argument(name + ".offset must be x, y, z in metres");
    arm->offset = Eigen::Vector3d(offset[0], offset[1], offset[2]);
    arm->publisher = create_publisher<Float64MultiArray>(arm->topic, 10);
    const auto input = prefix + "_" + std::to_string(arm->marker) + "/pose";
    // Depth 1: a pose that waited behind a slow TF lookup is not worth acting on.
    arm->subscription = create_subscription<PoseStamped>(
      input, 1, [this, arm](const PoseStamped & msg) { on_marker(*arm, msg); });
    arms_.push_back(arm);
    RCLCPP_INFO(get_logger(), "%s: marker %ld (%s) + [%.3f, %.3f, %.3f] in %s -> %s", name.c_str(), arm->marker,
                input.c_str(), offset[0], offset[1], offset[2], frame_.c_str(), arm->topic.c_str());
  }

  void on_marker(Arm & arm, const PoseStamped & msg) {
    arm.seen = true;
    Eigen::Isometry3d frame_from_camera;
    try {
      frame_from_camera = isometry(
        buffer_.lookupTransform(frame_, msg.header.frame_id, rclcpp::Time(msg.header.stamp),
                                rclcpp::Duration(kTfWait)).transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kQuietMs,
                           "%s: marker %ld is seen but its place in %s is unknown -- the robot's TF comes from a "
                           "target executor's launch, the camera's from camera.launch.py (%s)",
                           arm.name.c_str(), arm.marker, frame_.c_str(), error.what());
      return;
    }
    const Eigen::Vector3d point = rby1_apriltag::hand_point(
      frame_from_camera, Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), arm.offset);
    if (arm.sent && (point - *arm.sent).norm() < min_move_) return;
    if (arm.publisher->get_subscription_count() == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kQuietMs, "%s: no target executor listens on %s",
                           arm.name.c_str(), arm.topic.c_str());
      return;
    }
    if (!arm.orientation) {
      try {
        arm.orientation = isometry(buffer_.lookupTransform(frame_, arm.tool, tf2::TimePointZero).transform).rotation();
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kQuietMs, "%s: where %s is in %s is unknown (%s)",
                             arm.name.c_str(), arm.tool.c_str(), frame_.c_str(), error.what());
        return;
      }
    }
    Float64MultiArray target;
    const auto values = rby1_apriltag::target_values(*arm.orientation, point);
    target.data.assign(values.begin(), values.end());
    arm.publisher->publish(target);
    arm.sent = point;
    RCLCPP_INFO(get_logger(), "%s: target [%.3f, %.3f, %.3f] in %s (marker %ld)", arm.name.c_str(), point.x(),
                point.y(), point.z(), frame_.c_str(), arm.marker);
  }

  std::string frame_;
  double min_move_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::vector<std::shared_ptr<Arm>> arms_;
  rclcpp::TimerBase::SharedPtr waiting_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MarkerTarget>());
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(rclcpp::get_logger("marker_target"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
