// target_tag_filter: the AprilTags you care about, steadied, as poses and TF.
//
// Takes Isaac ROS AprilTag detections (/tag_detections), keeps the ids in
// `target_ids`, averages each over its last `window_size` frames (robust_average:
// translation median, rotation SVD mean) when `filter_jitter`, and publishes
//   /target_marker/pose            geometry_msgs/PoseStamped, every target in turn
//   /target_marker_<id>/pose       one topic per target (publish_per_tag_topics)
//   TF <camera frame> -> target_marker_<id>   (broadcast_tf, target_frame_prefix)
// in the camera's optical frame, so the head tracker and anything else can take them
// into the robot's frames through TF.
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rby1_apriltag/pose_average.hpp"

using isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

class TargetTagFilter : public rclcpp::Node {
public:
  TargetTagFilter() : rclcpp::Node("target_tag_filter") {
    const auto ids = declare_parameter<std::vector<int64_t>>("target_ids", {7});
    targets_.insert(ids.begin(), ids.end());
    const auto input = declare_parameter("input_topic", "/tag_detections");
    const auto output = declare_parameter("output_pose_topic", "/target_marker/pose");
    broadcast_tf_ = declare_parameter("broadcast_tf", true);
    prefix_ = declare_parameter("target_frame_prefix", "target_marker");
    filter_ = declare_parameter("filter_jitter", true);
    window_ = static_cast<size_t>(std::max<int64_t>(1, declare_parameter("window_size", 5)));
    per_tag_ = declare_parameter("publish_per_tag_topics", true);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output, 10);
    if (per_tag_) {
      for (const auto id : targets_) {
        per_tag_pubs_[id] = create_publisher<geometry_msgs::msg::PoseStamped>(
          "/" + prefix_ + "_" + std::to_string(id) + "/pose", 10);
      }
    }
    if (broadcast_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<AprilTagDetectionArray>(
      input, 10, [this](const AprilTagDetectionArray & msg) { on_detections(msg); });

    std::string list;
    for (const auto id : targets_) list += (list.empty() ? "" : ", ") + std::to_string(id);
    RCLCPP_INFO(get_logger(), "target ids [%s] from %s; smoothing %s", list.c_str(), input.c_str(),
                filter_ ? ("over " + std::to_string(window_) + " frames").c_str() : "off");
  }

private:
  void on_detections(const AprilTagDetectionArray & msg) {
    for (const auto & detection : msg.detections) {
      if (!targets_.count(detection.id)) continue;
      const auto & raw = detection.pose.pose.pose;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = Eigen::Quaterniond(raw.orientation.w, raw.orientation.x, raw.orientation.y,
                                         raw.orientation.z).normalized().toRotationMatrix();
      pose.translation() = Eigen::Vector3d(raw.position.x, raw.position.y, raw.position.z);
      if (filter_) {
        auto & history = history_[detection.id];
        history.push_back(pose);
        while (history.size() > window_) history.pop_front();
        pose = rby1_apriltag::robust_average(history);
      }
      const Eigen::Quaterniond q(pose.rotation());

      geometry_msgs::msg::PoseStamped out;
      out.header.stamp = msg.header.stamp;
      out.header.frame_id = rby1_apriltag::detection_frame(detection.pose.header.frame_id, msg.header.frame_id);
      out.pose.position.x = pose.translation().x();
      out.pose.position.y = pose.translation().y();
      out.pose.position.z = pose.translation().z();
      out.pose.orientation.x = q.x();
      out.pose.orientation.y = q.y();
      out.pose.orientation.z = q.z();
      out.pose.orientation.w = q.w();
      pose_pub_->publish(out);
      if (per_tag_) {
        auto & publisher = per_tag_pubs_[detection.id];
        if (!publisher) {
          publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/" + prefix_ + "_" + std::to_string(detection.id) + "/pose", 10);
        }
        publisher->publish(out);
      }
      if (broadcast_tf_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = out.header;
        transform.child_frame_id = prefix_ + "_" + std::to_string(detection.id);
        transform.transform.translation.x = out.pose.position.x;
        transform.transform.translation.y = out.pose.position.y;
        transform.transform.translation.z = out.pose.position.z;
        transform.transform.rotation = out.pose.orientation;
        tf_->sendTransform(transform);
      }
    }
  }

  std::set<int64_t> targets_;
  bool broadcast_tf_, filter_, per_tag_;
  std::string prefix_;
  size_t window_;
  std::map<int64_t, std::deque<Eigen::Isometry3d>> history_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::map<int64_t, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> per_tag_pubs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetTagFilter>());
  rclcpp::shutdown();
  return 0;
}
