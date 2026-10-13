cmake_minimum_required(VERSION 3.10)
project(rby1_isaac_ros_examples)

find_package(ament_cmake REQUIRED)

# Launch files only: each starts what the container runs for one use.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
