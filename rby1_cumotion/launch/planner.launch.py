"""GPU side; runs cuMotion against a self-contained model bundle."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rby1_cumotion.model import load_model, write_resolved_urdf


def setup(context):
    directory = Path(LaunchConfiguration('model_directory').perform(context)).resolve()
    metadata, root = load_model(directory, LaunchConfiguration('group').perform(context) or None)
    urdf_path = write_resolved_urdf(root)
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _context: Path(urdf_path).unlink(missing_ok=True))])),
        Node(
            package='isaac_ros_cumotion', executable='cumotion_planner_node',
            name='cumotion_planner', output='screen',
            parameters=[{
                'robot': metadata['xrdf_path'],
                'urdf_path': urdf_path,
                'tool_frame': metadata['tool_frame'],
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'read_esdf_world': False,
                'add_ground_plane': False,
                # time_dilation_factor is ignored unless override_moveit_scaling_factors
                # is true; MoveIt's request scaling wins. See cumotion_planner.py:649.
                'override_moveit_scaling_factors': False,
                'interpolation_dt': 0.025,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_directory', description='Bundle produced by prepare_model'),
        DeclareLaunchArgument('group', default_value='',
                              description='Planning group in the bundle; required when it holds several'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        OpaqueFunction(function=setup),
    ])
