"""The hand goes round the AprilTag markers in view, lowest id first: apriltag.launch.py, and marker_shuttle on top.

  ros2 launch rby1_apriltag marker_shuttle.launch.py [ids:=7,8,12] [size:=0.08]

Everything apriltag.launch.py does -- its arguments apply here unchanged -- and then
every `period` seconds (config/target_tags.yaml, marker_shuttle:) the next marker in
view becomes a 4x4 target on /rby1/<arm>/target_pose: with markers 7, 8 and 12 in view
7, 8, 12, 7, ... The markers are the ones the filter keeps (`ids`, else target_ids of
the config). A target executor (the cuMotion launch, or rby1_moveit_executor) plans to
each and sends the trajectory to the driver.

Needs the camera on the host (rby1_additional_tools camera.launch.py: images and the
camera mount's TF) and the robot's TF, which a target executor's launch publishes.

With a target executor running, this moves the robot. Do not run it together with
marker_target.launch.py: both would send targets to the same arm.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    config = LaunchConfiguration('config').perform(context)
    with open(config) as stream:
        data = yaml.safe_load(stream) or {}
    # apriltag.launch.py, run before this, has already refused ids that are not numbers.
    ids = LaunchConfiguration('ids').perform(context).strip().strip('[]')
    kept = ([int(part) for part in ids.replace(',', ' ').split()] if ids else
            list(((data.get('target_tag_filter') or {}).get('ros__parameters') or {}).get('target_ids') or []))
    if not kept:
        raise ValueError(f'no markers to go round: give ids:=, or target_ids in {config}')
    return [Node(package='rby1_apriltag', executable='marker_shuttle', name='marker_shuttle',
                 parameters=[config, {'marker_ids': kept}], output='screen')]


def generate_launch_description():
    share = get_package_share_directory('rby1_apriltag')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'apriltag.launch.py'))),
        OpaqueFunction(function=nodes),
    ])
