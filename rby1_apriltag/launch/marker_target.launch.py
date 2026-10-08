"""AprilTag markers as targets for the arms: apriltag.launch.py, and marker_target on top.

  ros2 launch rby1_apriltag marker_target.launch.py [width:=1280 height:=720] [ids:=7,8] [size:=0.08]

Everything apriltag.launch.py does -- its arguments apply here unchanged -- and then
each arm's marker in config/target_tags.yaml (marker_target: right_arm / left_arm:
marker_id, offset) becomes a 4x4 target on /rby1/right_arm/target_pose or
/rby1/left_arm/target_pose. A target executor (the cuMotion launch, or
rby1_moveit_executor) plans to it and sends the trajectory to the driver; nothing else
is needed in between.

Needs the camera on the host (rby1_additional_tools camera.launch.py: images and the
camera mount's TF) and the robot's TF, which a target executor's launch publishes.

With a target executor running, this moves the robot: a hand goes to its marker as soon
as the marker is seen, and again whenever the marker has moved.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARMS = ('right_arm', 'left_arm')


def unwatched_markers(config, ids):
    """Arms whose marker target_tag_filter would not pass on, as {arm: marker id}.

    `ids` is what the filter keeps: the launch's ids, else the config file's target_ids.
    """
    arms = (config.get('marker_target') or {}).get('ros__parameters') or {}
    markers = {arm: (arms.get(arm) or {}).get('marker_id') for arm in ARMS}
    return {arm: marker for arm, marker in markers.items()
            if marker is not None and marker >= 0 and marker not in ids}


def nodes(context):
    config = LaunchConfiguration('config').perform(context)
    with open(config) as stream:
        data = yaml.safe_load(stream) or {}
    # apriltag.launch.py, run before this, has already refused ids that are not numbers.
    ids = LaunchConfiguration('ids').perform(context).strip().strip('[]')
    kept = ([int(part) for part in ids.replace(',', ' ').split()] if ids else
            list(((data.get('target_tag_filter') or {}).get('ros__parameters') or {}).get('target_ids') or []))
    missing = unwatched_markers(data, kept)
    if missing:
        raise ValueError(f'marker_target follows markers the filter does not keep: {missing}, target ids {kept}. '
                         f'Add them to target_ids (or ids:=), or set that arm\'s marker_id to -1 ({config})')
    return [Node(package='rby1_apriltag', executable='marker_target', name='marker_target',
                 parameters=[config], output='screen')]


def generate_launch_description():
    share = get_package_share_directory('rby1_apriltag')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'apriltag.launch.py'))),
        OpaqueFunction(function=nodes),
    ])
