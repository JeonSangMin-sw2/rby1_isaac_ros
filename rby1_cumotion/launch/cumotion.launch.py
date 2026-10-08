"""The cuMotion stack for the RB-Y1: prepare, then planner, MoveIt and the target executor.

  ros2 launch rby1_cumotion cumotion.launch.py            # the stack, no windows
  ros2 launch rby1_cumotion demo.launch.py                # the same, with RViz

See rby1_cumotion/bringup.py for what starts and in which order.
"""

from rby1_cumotion.bringup import launch_description


def generate_launch_description():
    return launch_description(rviz_default=False)
