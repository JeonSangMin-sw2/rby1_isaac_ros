"""MoveIt + existing RBY1 controllers, with an optional local cuMotion server."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from rby1_cumotion.model import load_model, use_ros_mesh_uris


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    directory = Path(value('model_directory')).resolve()
    metadata, root = load_model(directory)
    use_ros_mesh_uris(root)
    fake = value('use_fake_hardware') == 'true'
    pipeline = value('pipeline')
    share = Path(get_package_share_directory('rby1_cumotion'))
    driver_share = Path(get_package_share_directory(f'rby1_moveit_{metadata["model"]}'))

    # Reuse the driver's hardware plugin and joint/controller names. Only one arm
    # controller is activated: no overlapping body/both_arms controller claims.
    control = ET.SubElement(root, 'ros2_control', name=metadata['robot_name'], type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = (
        'mock_components/GenericSystem' if fake else 'rby1_hardware/RBY1SystemHardware')
    if not fake:
        for key, val in {'robot_ip': value('robot_ip'), 'model': metadata['model'][0],
                         'driver_namespace': value('driver_namespace')}.items():
            ET.SubElement(hardware, 'param', name=key).text = val
    for name, initial in metadata['default_positions'].items():
        if not fake and name.startswith('gripper_'):
            continue  # The SDK stream does not command gripper fingers.
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='position')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(initial)
    description = {'robot_description': ET.tostring(root, encoding='unicode')}
    config = (MoveItConfigsBuilder(metadata['robot_name'], package_name=f'rby1_moveit_{metadata["model"]}')
              .robot_description_semantic(file_path=f'config/{metadata["robot_name"]}.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .to_moveit_configs())
    config.robot_description = description
    if pipeline == 'isaac_ros_cumotion':
        config.planning_pipelines['planning_pipelines'].append(pipeline)
        config.planning_pipelines[pipeline] = yaml.safe_load((share / 'config/isaac_ros_cumotion_planning.yaml').read_text())
    config.planning_pipelines['default_planning_pipeline'] = pipeline
    # Expose just the selected 7-DOF group to the planner and execution manager.
    semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    for child in list(semantic):
        if child.tag == 'group' and child.get('name') != metadata['group']:
            semantic.remove(child)
        elif child.tag in ('group_state', 'end_effector', 'virtual_joint'):
            semantic.remove(child)
    config.robot_description_semantic['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    config.robot_description_kinematics['robot_description_kinematics'] = {
        metadata['group']: config.robot_description_kinematics['robot_description_kinematics'][metadata['group']]}
    controller = f'{metadata["group"]}_controller'
    config.trajectory_execution['moveit_simple_controller_manager']['controller_names'] = [controller]
    nodes = []
    if value('start_state_publisher') == 'true':
        nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                          output='screen', parameters=[description]))
    if value('start_moveit') == 'true':
        nodes.append(Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[config.to_dict(), {'allow_trajectory_execution': True,
                         'publish_robot_description_semantic': True}]))
    if value('start_control') == 'true':
        # RBY1Hardware.read() resets unclaimed command targets to measured values.
        # A disjoint JTC holds the fixed joints at its activation posture, avoiding
        # gravity drift without sending a new posture goal to those joints.
        controllers = yaml.safe_load((driver_share / 'config/ros2_controllers.yaml').read_text())
        controllers['controller_manager']['ros__parameters']['locked_joints_controller'] = {
            'type': 'joint_trajectory_controller/JointTrajectoryController'}
        controllers['locked_joints_controller'] = {'ros__parameters': {
            'joints': list(metadata['locked_joints']), 'command_interfaces': ['position'],
            'state_interfaces': ['position'], 'allow_partial_joints_goal': False}}
        with tempfile.NamedTemporaryFile(mode='w', prefix='rby1_controllers_', suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(controllers, stream)
            controller_file = stream.name
        nodes.extend([
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
                function=lambda _context: Path(controller_file).unlink(missing_ok=True))])),
            Node(package='controller_manager', executable='ros2_control_node', output='screen',
                 parameters=[description, controller_file,
                             {'update_rate': 100}]),
            Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['joint_state_broadcaster', 'locked_joints_controller', controller,
                            '--controller-manager', '/controller_manager',
                            '--controller-manager-timeout', '60']),
        ])
    if value('start_planner') == 'true' and pipeline == 'isaac_ros_cumotion':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/planner.launch.py')),
            launch_arguments={'model_directory': str(directory)}.items()))
    if value('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', output='screen',
                          arguments=['-d', str(share / 'config/demo.rviz')],
                          parameters=[config.to_dict()]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_directory', description='Directory produced by prepare_model'),
        DeclareLaunchArgument('pipeline', default_value='isaac_ros_cumotion', choices=['isaac_ros_cumotion', 'ompl']),
        DeclareLaunchArgument('use_fake_hardware', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_control', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_moveit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_state_publisher', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_planner', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1:50051'),
        DeclareLaunchArgument('driver_namespace', default_value='rby1'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
