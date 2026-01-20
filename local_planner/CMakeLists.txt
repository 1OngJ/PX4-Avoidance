cmake_minimum_required(VERSION 3.5)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(perception_pcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(mavros_msgs REQUIRED)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(avoidance REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_SIMULATION")
endif()

#################
# Setup targets #
#################

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

include_directories(
  include
  ${avoidance_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
set(LOCAL_PLANNER_CPP_FILES
  "src/nodes/local_planner.cpp"
  "src/nodes/waypoint_generator.cpp"
  "src/nodes/tree_node.cpp"
  "src/nodes/star_planner.cpp"
  "src/nodes/planner_functions.cpp"
  "src/nodes/local_planner_visualization.cpp"
  "src/utils/trajectory_simulator.cpp"
)

# Add local_planner lib
add_library(local_planner SHARED "${LOCAL_PLANNER_CPP_FILES}")
ament_target_dependencies(local_planner
  rclcpp
  tf2
  tf2_ros
  tf2_sensor_msgs
  avoidance
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
)
target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(local_planner Eigen3::Eigen)
target_link_libraries(local_planner avoidance::avoidance)

## Declare a C++ executable
add_executable(local_planner_node
  src/nodes/local_planner_node_main.cpp
  src/nodes/local_planner_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  local_planner_node
  local_planner
  avoidance::avoidance
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Add node dependencies
ament_target_dependencies(
  local_planner_node
  rclcpp
  avoidance
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  mavros_msgs
)

# Export information to downstream packages
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_sensor_msgs
  avoidance
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
)
ament_export_targets(export_local_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(local_planner)

#############
## Install ##
#############

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

# Install artifacts
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  install(DIRECTORY cmake
    DESTINATION share/${PROJECT_NAME}
  )
endif()
install(TARGETS local_planner
        EXPORT export_local_planner
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(TARGETS local_planner_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## MAVROS TF Publisher - bridges MAVROS pose to TF tree
add_executable(mavros_tf_publisher
  src/nodes/mavros_tf_publisher.cpp
)
target_link_libraries(mavros_tf_publisher
  ${Boost_LIBRARIES}
)
ament_target_dependencies(mavros_tf_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)
install(TARGETS mavros_tf_publisher
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  params
  resource
  DESTINATION share/${PROJECT_NAME}
)

# Install test scripts
install(PROGRAMS
  test/test_node_interfaces.py
  test/test_message_flow.py
  test/test_mission_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  find_package(ament_cmake_gtest REQUIRED)

  # TODO: Port legacy ROS1 tests to ROS2
  # The original test files use ros::Time and geometry_msgs::PoseStamped (ROS1 style)
  # They need to be updated to use rclcpp::Time and geometry_msgs::msg::PoseStamped

  # # 基础算法单元测试（不依赖 ROS 节点）
  # ament_add_gtest(test_planner_functions
  #   test/main.cpp
  #   test/test_planner_functions.cpp
  # )
  # if(TARGET test_planner_functions)
  #   target_link_libraries(test_planner_functions local_planner)
  #   ament_target_dependencies(test_planner_functions rclcpp)
  # endif()

  # ament_add_gtest(test_trajectory_simulator
  #   test/main.cpp
  #   test/test_trajectory_simulator.cpp
  # )
  # if(TARGET test_trajectory_simulator)
  #   target_link_libraries(test_trajectory_simulator local_planner)
  #   ament_target_dependencies(test_trajectory_simulator rclcpp)
  # endif()
endif()

# Legacy catkin tests (commented out, kept for reference)
# if(CATKIN_ENABLE_TESTING)
#     # Add gtest based cpp test target and link libraries
#     catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
#                                           test/test_example.cpp
#                                           test/test_local_planner.cpp
#                                           test/test_planner_functions.cpp
#                                           test/test_star_planner.cpp
#                                           test/test_trajectory_simulator.cpp
#                                           test/test_waypoint_generator.cpp)
#
#     catkin_add_gtest(${PROJECT_NAME}-test-roscore test/main.cpp
#                                         test/test_local_planner_nodelet.cpp)
#     if(TARGET ${PROJECT_NAME}-test)
#         target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
#                                                  ${catkin_LIBRARIES}
#                                                  ${YAML_CPP_LIBRARIES})
#     endif()
#
#     if(TARGET ${PROJECT_NAME}-test-roscore)
#         target_link_libraries(${PROJECT_NAME}-test-roscore local_planner_nodelet
#                                                  ${catkin_LIBRARIES}
#                                                  ${YAML_CPP_LIBRARIES})
#     endif()
#
#
#     if (${CMAKE_BUILD_TYPE} STREQUAL "Coverage")
#         SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage --coverage")
#         SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage --coverage")
#         SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
#
#         add_custom_target(${PROJECT_NAME}-test_coverage
#             COMMAND lcov --zerocounters --directory ${PROJECT_BINARY_DIR}
#             COMMAND lcov --capture  --initial --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${${PROJECT_NAME}_SOURCE_DIR} --output-file base_coverage.info --rc lcov_branch_coverage=1
#             COMMAND ${PROJECT_NAME}-test
#             COMMAND ${PROJECT_NAME}-test-roscore
#             COMMAND lcov --capture  --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${${PROJECT_NAME}_SOURCE_DIR} --output-file test_coverage.info --rc lcov_branch_coverage=1
#             COMMAND lcov -a base_coverage.info -a test_coverage.info -o coverage.info --rc lcov_branch_coverage=1
#             COMMAND lcov --rc lcov_branch_coverage=1 --summary coverage.info
#             WORKING_DIRECTORY .
#             DEPENDS ${PROJECT_NAME}-test ${PROJECT_NAME}-test-roscore
#         )
#         add_custom_target(${PROJECT_NAME}-test_coverage_html
#             COMMAND genhtml coverage.info --output-directory out --branch-coverage
#             COMMAND x-www-browser out/index.html
#             WORKING_DIRECTORY .
#             DEPENDS ${PROJECT_NAME}-test_coverage
#         )
#     endif()
#
#     ## Add folders to be run by python nosetests
#     # catkin_add_nosetests(test)
# endif()

ament_package()
