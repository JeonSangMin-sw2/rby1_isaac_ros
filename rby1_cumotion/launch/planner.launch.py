"""GPU side; use artifacts generated inside this container's filesystem."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rby1_cumotion.model import load_model


def setup(context):
    directory = Path(LaunchConfiguration('model_directory').perform(context)).resolve()
    metadata, root = load_model(directory)
    for mesh in root.findall('.//mesh'):
        if not Path(mesh.get('filename')).is_file():
            raise RuntimeError('Model mesh is inaccessible; run prepare_model inside this environment')
    return [Node(
        package='isaac_ros_cumotion', executable='cumotion_planner_node',
        name='cumotion_planner', output='screen',
        parameters=[{
            'robot': str(directory / 'robot.xrdf'),
            'urdf_path': str(directory / 'robot.urdf'),
            'tool_frame': metadata['tool_frame'],
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'read_esdf_world': False,
            'add_ground_plane': False,
            'time_dilation_factor': 0.1,
            'override_moveit_scaling_factors': False,
            'interpolation_dt': 0.025,
            'grid_center_m': [0.0, 0.0, 1.0],
            'grid_size_m': [2.0, 2.0, 2.0],
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_directory', description='Directory produced by prepare_model'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        OpaqueFunction(function=setup),
    ])
