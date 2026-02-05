cmake_minimum_required(VERSION 3.8)
project(avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation")
  add_compile_definitions(DISABLE_SIMULATION)
else()
  message(STATUS "Building avoidance with Gazebo Simulation")
  find_package(yaml-cpp REQUIRED)
endif()

#################
# Setup targets #
#################

# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Declare a C++ library
set(AVOIDANCE_CPP_FILES
  "src/common.cpp"
  "src/histogram.cpp"
  "src/transform_buffer.cpp"
  "src/avoidance_node.cpp"
)

if(NOT DISABLE_SIMULATION)
  list(APPEND AVOIDANCE_CPP_FILES "src/rviz_world_loader.cpp")
endif()

add_library(avoidance SHARED ${AVOIDANCE_CPP_FILES})

target_include_directories(avoidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(avoidance
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(avoidance
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# Export information to downstream packages
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  eigen3_cmake_module
  Eigen3
)
ament_export_targets(export_avoidance HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(avoidance)

#############
## Install ##
#############

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

# Install library
install(TARGETS avoidance
        EXPORT export_avoidance
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  resource
  sim
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Add gtest based cpp test target
  ament_add_gtest(${PROJECT_NAME}-test
    test/main.cpp
    test/test_common.cpp
    test/test_usm.cpp
    test/test_transform_buffer.cpp
  )

  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test
      ${PROJECT_NAME}
      ${YAML_CPP_LIBRARIES}
    )
    ament_target_dependencies(${PROJECT_NAME}-test rclcpp)
  endif()
endif()

ament_package()
