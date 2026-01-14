cmake_minimum_required(VERSION 3.8)
project(local_planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(avoidance_ros2 REQUIRED)

add_library(local_planner_ros2_component SHARED
  src/local_planner_component.cpp
)

target_include_directories(local_planner_ros2_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(local_planner_ros2_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  avoidance_ros2
)

rclcpp_components_register_nodes(local_planner_ros2_component "local_planner_ros2::LocalPlannerComponent")

install(TARGETS local_planner_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
