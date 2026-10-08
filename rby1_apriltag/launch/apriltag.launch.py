"""AprilTag markers in images from the host camera, on the GPU (Isaac ROS).

  ros2 launch rby1_apriltag apriltag.launch.py [width:=1280 height:=720] [ids:=7,12] [size:=0.10]

The camera runs on the host (rby1_additional_tools camera.launch.py) and publishes
/camera/image_raw and /camera/camera_info; nothing here touches a device. The images
are rectified (isaac_ros_image_proc), searched for tags (isaac_ros_apriltag: all
detections on /tag_detections), and the ones in config/target_tags.yaml are
smoothed and published as /target_marker/pose and TF target_marker_<id>
(target_tag_filter).

The markers are set in config/target_tags.yaml: the edge of the tag's black square
in metres (`size` -- a tag's pose comes from that size and the camera model, from
one image; no depth is used), the family, and the target ids. `size`, `tag_family`
and `ids` given here override the file. `width`/`height` must be the camera's
image size.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def target_ids(text):
    """'7,12' or '[7, 12]' as [7, 12]; '' as None (the config file's)."""
    text = text.strip().strip('[]')
    if not text:
        return None
    try:
        return [int(part) for part in text.replace(',', ' ').split()]
    except ValueError:
        raise ValueError(f'ids must be marker numbers separated by commas, got {text!r}') from None


def read_config(path):
    with open(path) as stream:
        return yaml.safe_load(stream) or {}


def marker_settings(path, size='', family=''):
    """size (m) and tag_family for the AprilTag node: the launch arguments, else the config file's."""
    data = read_config(path)
    settings = dict((data.get('apriltag') or {}).get('ros__parameters') or {})
    if size.strip():
        settings['size'] = size
    if family.strip():
        settings['tag_family'] = family
    if 'size' not in settings or 'tag_family' not in settings:
        raise ValueError(f'{path} has no apriltag: ros__parameters: size / tag_family (the edge of the tag\'s '
                         'black square in metres, and the family) -- add them, or pass size:= and tag_family:=')
    try:
        settings['size'] = float(settings['size'])
    except (TypeError, ValueError):
        raise ValueError(f'size must be a length in metres, got {settings["size"]!r}') from None
    if not 0.0 < settings['size'] < 10.0:
        raise ValueError(f'size is the edge of the tag\'s black square in metres, got {settings["size"]}')
    return {'size': settings['size'], 'tag_family': str(settings['tag_family'])}


def nodes(context):
    value = LaunchConfiguration
    config = value('config').perform(context)
    marker = marker_settings(config, value('size').perform(context), value('tag_family').perform(context))
    rectify = ComposableNode(
        package='isaac_ros_image_proc', plugin='nvidia::isaac_ros::image_proc::RectifyNode', name='rectify',
        parameters=[{'output_width': value('width'), 'output_height': value('height')}],
        remappings=[('image_raw', value('image')), ('camera_info', value('camera_info'))])
    apriltag = ComposableNode(
        package='isaac_ros_apriltag', plugin='nvidia::isaac_ros::apriltag::AprilTagNode', name='apriltag',
        parameters=[marker],
        remappings=[('image', 'image_rect'), ('camera_info', 'camera_info_rect')])
    container = ComposableNodeContainer(
        package='rclcpp_components', executable='component_container_mt', name='apriltag_container',
        namespace='', composable_node_descriptions=[rectify, apriltag], output='screen')
    parameters = [config]
    ids = target_ids(value('ids').perform(context))
    if ids is not None:
        parameters.append({'target_ids': ids})
    target_filter = Node(package='rby1_apriltag', executable='target_tag_filter', name='target_tag_filter',
                         parameters=parameters, output='screen')
    if ids is None:
        ids = ((read_config(config).get('target_tag_filter') or {}).get('ros__parameters') or {}).get('target_ids')
    # What is used and from which file: the installed copy of the config is not the one in
    # the source tree unless the package was built with --symlink-install.
    return [LogInfo(msg=f'markers: {marker["tag_family"]}, black square {marker["size"]:g} m, target ids {ids} '
                        f'(settings: {os.path.realpath(config)})'),
            container, target_filter]


def generate_launch_description():
    share = get_package_share_directory('rby1_apriltag')
    return LaunchDescription([
        DeclareLaunchArgument('image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info', default_value='/camera/camera_info'),
        DeclareLaunchArgument('width', default_value='1280', description='camera image width'),
        DeclareLaunchArgument('height', default_value='720', description='camera image height'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'target_tags.yaml'),
                              description='marker settings: size, tag_family, target_ids, smoothing'),
        DeclareLaunchArgument('size', default_value='',
                              description='tag black-square edge in m (empty: size of the config)'),
        DeclareLaunchArgument('tag_family', default_value='', description='empty: tag_family of the config'),
        DeclareLaunchArgument('ids', default_value='',
                              description='target marker ids, comma separated (empty: target_ids of the config)'),
        OpaqueFunction(function=nodes),
    ])
