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
find_package(tf2_ros REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_core STATIC src/pose_average.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})

add_executable(target_tag_filter src/target_tag_filter.cpp)
target_link_libraries(target_tag_filter ${PROJECT_NAME}_core)
ament_target_dependencies(target_tag_filter rclcpp geometry_msgs tf2_ros isaac_ros_apriltag_interfaces)

install(TARGETS target_tag_filter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_average test/test_pose_average.cpp)
  target_link_libraries(test_pose_average ${PROJECT_NAME}_core)
endif()

ament_package()
