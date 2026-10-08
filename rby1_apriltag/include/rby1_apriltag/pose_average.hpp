// Steadying a marker pose that jitters from frame to frame.
#pragma once

#include <deque>
#include <string>

#include <Eigen/Geometry>

namespace rby1_apriltag {

// The robust mean of camera-to-marker transforms: translation by the component-wise
// median (one wild frame does not drag it), rotation by the SVD projection of the
// summed rotation matrices onto the nearest proper rotation.
Eigen::Isometry3d robust_average(const std::deque<Eigen::Isometry3d> & transforms);

// The frame a detection's pose is in: its own header's, or -- Isaac ROS AprilTag
// leaves that empty -- the detection array's (the camera's optical frame).
std::string detection_frame(const std::string & pose_frame, const std::string & array_frame);

}  // namespace rby1_apriltag
