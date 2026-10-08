"""The cuMotion stack for the RB-Y1, shared by cumotion.launch.py and demo.launch.py.

Two stages, because move_group and the cuMotion planner load the robot model
when they start:

  1. prepare (rby1_cumotion/bringup/prepare.py) runs once: driver check, robot kind and
     version -> bundle, emergency stop and faults, power and servos, a straight
     planning arm to the ready pose, then the runtime bundle with the posture
     locked where the robot now stands. It exits.
  2. Only if it succeeded: robot_state_publisher, the joint-state relay,
     move_group (planning and the planning scene only), the cuMotion planner and
     the target executor. demo.launch.py adds RViz.

Every default comes from config/cumotion.yaml (config:=... for another copy);
the arguments below override single entries. hardware:=mock is for development
without a driver: ros2_control with mock hardware, no executor.
"""

import logging
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from rby1_cumotion import planner_params, settings
from rby1_cumotion.model import ACTIVE_BUNDLE, active_record, load_model, use_ros_mesh_uris

PREPARE_ARGUMENTS = ('config', 'hardware', 'model', 'model_directory', 'group', 'driver_namespace')


def config_path(value):
    return value('config') or str(settings.default_path())


def setup(context):
    """The planning stack, built from the runtime bundle prepare just made."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    directory = ACTIVE_BUNDLE
    record = active_record(directory)
    mock = record['hardware'] == 'mock'
    metadata, root = load_model(directory)
    use_ros_mesh_uris(root)
    pipeline = value('pipeline')
    share = Path(get_package_share_directory('rby1_cumotion'))

    if mock:
        # Mock hardware only. The driver path uses no ros2_control at all: the
        # driver refuses its trajectory action while hardware control is claimed.
        control = ET.SubElement(root, 'ros2_control', name=metadata['robot_name'], type='system')
        hardware = ET.SubElement(control, 'hardware')
        ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
        for name, initial in metadata['default_positions'].items():
            joint = ET.SubElement(control, 'joint', name=name)
            ET.SubElement(joint, 'command_interface', name='position')
            state = ET.SubElement(joint, 'state_interface', name='position')
            ET.SubElement(state, 'param', name='initial_value').text = str(initial)
    description = {'robot_description': ET.tostring(root, encoding='unicode')}
    # Every file comes from the bundle. package_name only anchors MoveIt's own
    # pipeline defaults: absolute file_paths bypass the package share directory,
    # and neither rby1_moveit_* nor rby1_description needs to be installed here.
    # Constructing the builder warns that it cannot infer a URDF/SRDF from that
    # package. It is not meant to: both are supplied explicitly just below.
    root_logger = logging.getLogger()
    level = root_logger.level
    root_logger.setLevel(logging.ERROR)
    try:
        config = (MoveItConfigsBuilder(metadata['robot_name'], package_name='rby1_cumotion')
                  .robot_description_semantic(file_path=str(directory / 'robot.srdf'))
                  .robot_description_kinematics(file_path=str(directory / 'kinematics.yaml'))
                  .joint_limits(file_path=str(directory / 'joint_limits.yaml'))
                  .planning_pipelines(pipelines=['ompl'])
                  .trajectory_execution(file_path=str(directory / 'moveit_controllers.yaml'))
                  .to_moveit_configs())
    finally:
        root_logger.setLevel(level)
    config.robot_description = description
    if pipeline == 'isaac_ros_cumotion':
        config.planning_pipelines['planning_pipelines'].append(pipeline)
        planning = share / 'config/isaac_ros_cumotion_planning.yaml'
        config.planning_pipelines[pipeline] = yaml.safe_load(planning.read_text())
    config.planning_pipelines['default_planning_pipeline'] = pipeline
    # Expose only the groups this bundle plans for, to the planner and the
    # execution manager. A multi-part group (an arm plus the torso) is published
    # as a group of those SRDF groups, like the driver's own both_arms.
    parts = metadata['srdf_groups']
    semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    for child in list(semantic):
        if child.tag == 'group' and child.get('name') not in parts:
            semantic.remove(child)
        elif child.tag in ('group_state', 'end_effector', 'virtual_joint'):
            semantic.remove(child)
    if metadata['group'] not in parts:
        composite = ET.SubElement(semantic, 'group', name=metadata['group'])
        for part in parts:
            ET.SubElement(composite, 'group', name=part)
    config.robot_description_semantic['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    # A composite group has no single chain, so it gets no IK solver -- the same
    # arrangement the driver ships for both_arms. cuMotion runs its own IK; OMPL
    # can only take joint-space goals for such a group.
    kinematics = config.robot_description_kinematics['robot_description_kinematics']
    config.robot_description_kinematics['robot_description_kinematics'] = {
        name: entry for name, entry in kinematics.items() if name in parts}
    # With the driver, execution goes to the driver directly and move_group gets
    # allow_trajectory_execution false below, so it never builds a controller
    # manager. The names stay set either way: an empty list is not a valid ROS
    # parameter (its element type cannot be inferred) and fails the launch.
    config.trajectory_execution['moveit_simple_controller_manager']['controller_names'] = \
        metadata['controllers']
    nodes = []
    if not mock:
        source = f'/{record["driver_namespace"].strip("/")}/joint_states'
        nodes.append(Node(package='rby1_cumotion', executable='joint_state_relay',
                          output='screen', parameters=[{'source': source,
                                                        'target': '/joint_states'}]))
    if value('start_state_publisher') == 'true':
        nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                          output='screen', parameters=[description]))
    if value('start_moveit') == 'true':
        nodes.append(Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                          parameters=[config.to_dict(), {'allow_trajectory_execution': mock,
                                                         'publish_robot_description_semantic': True}]))
    if mock:
        # RBY1Hardware.read() resets unclaimed command targets to measured values.
        # A disjoint JTC holds the fixed joints at its activation posture, avoiding
        # gravity drift without sending a new posture goal to those joints.
        controllers = yaml.safe_load((directory / 'ros2_controllers.yaml').read_text())
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
                 arguments=['joint_state_broadcaster', 'locked_joints_controller',
                            *metadata['controllers'],
                            '--controller-manager', '/controller_manager',
                            '--controller-manager-timeout', '60']),
        ])
    if value('start_planner') == 'true' and pipeline == 'isaac_ros_cumotion':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/planner.launch.py')),
            launch_arguments={'model_directory': str(directory),
                              'group': metadata['group'],
                              # Forward every planner setting untouched; defaults
                              # live in planner_params, not restated here.
                              **{name: value(name) for name in planner_params.launch_arguments()},
                              }.items()))
    if not mock and value('start_executor') == 'true':
        nodes.append(Node(package='rby1_cumotion', executable='target_executor', output='screen',
                          parameters=[{'config': config_path(value),
                                       'pipeline': pipeline,
                                       'driver_namespace': record['driver_namespace']}]))
    if value('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', output='screen',
                          arguments=['-d', str(share / 'config/demo.rviz')],
                          parameters=[config.to_dict()]))
    return nodes


def after_prepare(event, context):
    if event.returncode != 0:
        return [LogInfo(msg='cuMotion prepare failed (see PREPARE_FAILED above); nothing else was started'),
                EmitEvent(event=Shutdown(reason='cumotion prepare failed'))]
    return setup(context)


def launch_description(rviz_default):
    prepare = Node(
        package='rby1_cumotion', executable='prepare', name='cumotion_prepare', output='screen',
        arguments=[item for name in PREPARE_ARGUMENTS
                   for item in (f'--{name.replace("_", "-")}', LaunchConfiguration(name))])
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='',
                              description='Bundle under $RBY1_BUNDLES, e.g. m_1_2. Empty: from the '
                                          'driver, or $RBY1_MODEL / m_1_2 with hardware:=mock'),
        DeclareLaunchArgument('model_directory', default_value='',
                              description='Explicit bundle path; overrides model'),
        DeclareLaunchArgument('group', default_value='',
                              description='Planning group, e.g. right_arm or right_arm+torso. '
                                          'Empty: robot.group in the config'),
        DeclareLaunchArgument('pipeline', default_value='isaac_ros_cumotion', choices=['isaac_ros_cumotion', 'ompl']),
        DeclareLaunchArgument('hardware', default_value='driver', choices=['driver', 'mock'],
                              description='driver: the host RB-Y1 driver (simulator or real '
                                          'robot, by its robot_ip). mock: development without '
                                          'a driver, planning only'),
        DeclareLaunchArgument('driver_namespace', default_value='',
                              description='Empty: robot.driver_namespace in the config'),
        DeclareLaunchArgument('start_moveit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_state_publisher', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_planner', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true' if rviz_default else 'false',
                              choices=['true', 'false']),
        *planner_params.declare(DeclareLaunchArgument),
        DeclareLaunchArgument('start_executor', default_value='true', choices=['true', 'false'],
                              description='Start target_executor (driver only)'),
        prepare,
        RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=after_prepare)),
    ])
