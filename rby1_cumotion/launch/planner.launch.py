"""GPU side; runs cuMotion against a self-contained model bundle.

Planner defaults come from the planner section of config/cumotion.yaml; a launch argument
overrides one entry. A bad value is rejected here, not at the first query.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from rby1_cumotion import planner_params
from rby1_cumotion.model import load_model, write_resolved_urdf


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    directory = Path(value('model_directory')).resolve()
    metadata, root = load_model(directory, value('group') or None)
    # Validate before writing the temporary URDF, so a bad value leaves nothing behind.
    default_config = Path(get_package_share_directory('rby1_cumotion')) / 'config' / \
        planner_params.CONFIG_NAME
    settings = planner_params.resolve(value, default_config)
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
                'joint_states_topic': value('joint_states_topic'),
                **settings,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_directory', description='Bundle produced by prepare_model'),
        DeclareLaunchArgument('group', default_value='',
                              description='Planning group in the bundle; required when it holds several'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        *planner_params.declare(DeclareLaunchArgument),
        OpaqueFunction(function=setup),
    ])
