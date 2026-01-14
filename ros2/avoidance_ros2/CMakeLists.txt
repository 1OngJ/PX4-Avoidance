cmake_minimum_required(VERSION 3.8)
project(avoidance_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(avoidance_ros2 SHARED
  src/px4_param_bridge.cpp
)

target_include_directories(avoidance_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(avoidance_ros2
  rclcpp
  rclcpp_components
  rcl_interfaces
  mavros_msgs
  std_msgs
)

rclcpp_components_register_nodes(avoidance_ros2 "avoidance_ros2::Px4ParamBridge")

install(TARGETS avoidance_ros2
  EXPORT export_avoidance_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(EXPORT export_avoidance_ros2
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_targets(export_avoidance_ros2)

ament_package()
