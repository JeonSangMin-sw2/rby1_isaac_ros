"""cumotion.launch.py plus RViz: the stack with the robot, the planning scene and the plans shown.

  ros2 launch rby1_cumotion demo.launch.py
  ros2 launch rby1_cumotion demo.launch.py group:=left_arm
  ros2 launch rby1_cumotion demo.launch.py hardware:=mock start_executor:=false   # development

See rby1_cumotion/bringup.py.
"""

from rby1_cumotion.bringup import launch_description


def generate_launch_description():
    return launch_description(rviz_default=True)
