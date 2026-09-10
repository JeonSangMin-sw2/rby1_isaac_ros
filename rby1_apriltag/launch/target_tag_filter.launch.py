import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rby1_apriltag')
    default_config = os.path.join(pkg_share, 'config', 'target_tags.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to configuration yaml file'
    )

    target_filter_node = Node(
        package='rby1_apriltag',
        executable='target_tag_filter',
        name='target_tag_filter',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        target_filter_node
    ])
