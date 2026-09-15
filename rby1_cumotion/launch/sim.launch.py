"""Real ROS driver -> local Docker simulator -> MoveIt and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rby1_cumotion.model import load_model


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    directory = str(Path(value('model_directory')).resolve())
    metadata, _ = load_model(directory)
    if metadata['model'] != 'm_1_2':
        raise ValueError('sim.launch.py requires an M v1.2 model')
    share = Path(get_package_share_directory('rby1_cumotion'))
    driver_share = Path(get_package_share_directory('rby1_driver'))
    driver = Node(package='rby1_driver', executable='rby1_ros2_driver',
                  name='rby1_ros2_driver', namespace='rby1', output='screen',
                  prefix=['chrt -f 1'],
                  parameters=[str(driver_share / 'config/driver_parameters.yaml'),
                              {'robot_ip': '127.0.0.1:50051', 'model': 'm'}])
    ready = Node(package='rby1_cumotion', executable='prepare_sim', output='screen',
                 parameters=[{'model_directory': directory, 'container': value('container')}])
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/demo.launch.py')),
        launch_arguments={'model_directory': directory, 'pipeline': value('pipeline'),
                          'start_moveit': value('start_moveit'),
                          'start_planner': value('start_planner'), 'rviz': value('rviz'),
                          'use_fake_hardware': 'false', 'robot_ip': '127.0.0.1:50051'}.items())

    def after_ready(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Simulator initialization failed'))]
        return [LogInfo(msg='Simulator posture verified; starting MoveIt and SDK hardware'), demo]

    return [RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)),
            RegisterEventHandler(OnProcessExit(target_action=driver,
                on_exit=[EmitEvent(event=Shutdown(reason='RBY1 driver exited'))])), driver, ready]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_directory'),
        DeclareLaunchArgument('container', default_value='rby1-cumotion-sim'),
        DeclareLaunchArgument('pipeline', default_value='ompl', choices=['ompl', 'isaac_ros_cumotion']),
        DeclareLaunchArgument('start_planner', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_moveit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
