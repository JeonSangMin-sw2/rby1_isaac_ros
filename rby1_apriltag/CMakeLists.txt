cmake_minimum_required(VERSION 3.10)
project(rby1_apriltag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rby1_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_core STATIC src/pose_average.cpp src/hand_target.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})

add_executable(target_tag_filter src/target_tag_filter.cpp)
target_link_libraries(target_tag_filter ${PROJECT_NAME}_core)
ament_target_dependencies(target_tag_filter rclcpp geometry_msgs tf2_ros isaac_ros_apriltag_interfaces)

add_executable(marker_target src/marker_target.cpp)
target_link_libraries(marker_target ${PROJECT_NAME}_core)
ament_target_dependencies(marker_target rclcpp geometry_msgs std_msgs tf2_ros)

add_executable(head_follow src/head_follow.cpp)
target_link_libraries(head_follow ${PROJECT_NAME}_core)
ament_target_dependencies(head_follow rclcpp rclcpp_action geometry_msgs sensor_msgs rby1_msgs)

install(TARGETS target_tag_filter marker_target head_follow DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_average test/test_pose_average.cpp)
  target_link_libraries(test_pose_average ${PROJECT_NAME}_core)
  ament_add_gtest(test_hand_target test/test_hand_target.cpp)
  target_link_libraries(test_hand_target ${PROJECT_NAME}_core)
  # head_follow.cpp without its node and main: the logic only, no ROS.
  ament_add_gtest(test_head_follow test/test_head_follow.cpp src/head_follow.cpp)
  target_compile_definitions(test_head_follow PRIVATE HEAD_FOLLOW_NO_MAIN)
  target_link_libraries(test_head_follow ${PROJECT_NAME}_core)
endif()

ament_package()
