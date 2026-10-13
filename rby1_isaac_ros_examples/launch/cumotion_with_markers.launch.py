"""Marker detection and cuMotion with one command: everything the container runs.

  ros2 launch rby1_isaac_ros_examples cumotion_with_markers.launch.py

How it starts is set by the values at the top of this file (below the imports): edit
them and start it again, no arguments needed. Each is also a launch argument of the
same name in lower case, for one run: rviz:=true marker_target:=true follow_head:=true ids:=7,8

With the values as shipped nothing moves yet: the markers are found and cuMotion waits
for a target. The robot moves when something sends one on /rby1/<arm>/target_pose --
your own node, rby1_examples 16_target_shuttle_publisher on the host, or one of the
two marker nodes of rby1_apriltag: with MARKER_TARGET marker_target (each arm's marker
in the tags config, with its offset, becomes that hand's target), with MARKER_SHUTTLE
marker_shuttle (one hand goes round the markers in view, lowest id first).

Starts two launch files of this repository, unchanged:

  rby1_apriltag  apriltag.launch.py   images from the host camera -> marker poses on
                                      /rby1/marker_<id>/pose and TF target_marker_<id>
                                      (marker_target.launch.py with MARKER_TARGET)
  rby1_cumotion  cumotion.launch.py   prepare, the cuMotion planner, MoveIt and the
                                      target executor (/rby1/<arm>/target_pose)

On the host only the driver and the camera node are needed:

  ros2 launch rby1_driver rby1_ros2_driver.launch.py
  ros2 launch rby1_additional_tools camera.launch.py     (a RealSense: camera:=realsense)

The other arguments of the two launch files work here too (width, height, size,
group, model, ...). Only `config` is in both, so it is given as tags_config
(rby1_apriltag's target_tags.yaml) and cumotion_config (rby1_cumotion's cumotion.yaml);
a plain config:= is not used.

Moves the robot: cuMotion's launch turns power and servos on and takes straight arms
to the ready pose; FOLLOW_HEAD turns the head after its marker; with MARKER_TARGET a
hand goes to its marker as soon as the marker is seen.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Values to change: edit these to start the example differently.
RVIZ = 'false'           # 'true': RViz with the robot, the obstacles and the plans
MARKER_TARGET = 'false'  # 'true': each arm's marker becomes its hand's target -- the hand goes to the marker
MARKER_SHUTTLE = 'false'  # 'true': one hand goes round the markers in view, lowest id first (not with MARKER_TARGET)
FOLLOW_HEAD = ''         # 'true' / 'false': the head follows its marker; '': head_follow: enabled of the tags config
IDS = ''                 # the markers to keep, e.g. '7,8'; '': target_ids of the tags config
TAGS_CONFIG = ''         # a marker settings file; '': rby1_apriltag's config/target_tags.yaml
CUMOTION_CONFIG = ''     # a cuMotion settings file; '': rby1_cumotion's config/cumotion.yaml


def markers(context):
    """The marker launch file for what was asked: detection alone, or with one of the two that send targets."""
    target = LaunchConfiguration('marker_target').perform(context) == 'true'
    shuttle = LaunchConfiguration('marker_shuttle').perform(context) == 'true'
    if target and shuttle:
        raise ValueError('marker_target and marker_shuttle both send targets to the arm: turn one of them off')
    name = 'marker_shuttle' if shuttle else 'marker_target' if target else 'apriltag'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rby1_apriltag'), 'launch', f'{name}.launch.py')),
        launch_arguments={'config': LaunchConfiguration('tags_config')}.items())]


def generate_launch_description():
    apriltag = get_package_share_directory('rby1_apriltag')
    cumotion = get_package_share_directory('rby1_cumotion')
    tags = TAGS_CONFIG or os.path.join(apriltag, 'config', 'target_tags.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value=RVIZ, choices=['true', 'false']),
        DeclareLaunchArgument('marker_target', default_value=MARKER_TARGET, choices=['true', 'false'],
                              description='true: each arm\'s marker becomes its hand\'s target '
                                          '(marker_target.launch.py instead of apriltag.launch.py): the hand '
                                          'goes to the marker as soon as it is seen'),
        # Arguments of apriltag.launch.py, declared here for the values above: it takes what is set.
        DeclareLaunchArgument('follow_head', default_value=FOLLOW_HEAD,
                              description='true: the head follows its marker (empty: the tags config decides)'),
        DeclareLaunchArgument('ids', default_value=IDS,
                              description='target marker ids, comma separated (empty: the tags config\'s)'),
        DeclareLaunchArgument('tags_config', default_value=tags,
                              description='Marker settings (config of apriltag.launch.py)'),
        DeclareLaunchArgument('cumotion_config', default_value=CUMOTION_CONFIG,
                              description='cuMotion settings (config of cumotion.launch.py); empty: the '
                                          'installed cumotion.yaml'),
        DeclareLaunchArgument('marker_shuttle', default_value=MARKER_SHUTTLE, choices=['true', 'false'],
                              description='true: one hand goes round the markers in view, lowest id first '
                                          '(marker_shuttle.launch.py). Not together with marker_target'),
        # In a group of its own: its `config` must not be the one cuMotion's launch reads.
        GroupAction([OpaqueFunction(function=markers)]),
        # Not in a group: this launch reads its arguments again when its prepare step has ended.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cumotion, 'launch', 'cumotion.launch.py')),
            launch_arguments={'config': LaunchConfiguration('cumotion_config'),
                              'rviz': LaunchConfiguration('rviz')}.items()),
    ])
