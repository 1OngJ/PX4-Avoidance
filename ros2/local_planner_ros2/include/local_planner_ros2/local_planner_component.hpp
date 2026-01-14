#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/trajectory.hpp>
#include <mavros_msgs/msg/altitude.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <avoidance_ros2/model_parameters.hpp>

namespace local_planner_ros2 {

class LocalPlannerComponent : public rclcpp::Node {
 public:
  explicit LocalPlannerComponent(const rclcpp::NodeOptions & options);
  ~LocalPlannerComponent() override;

 private:
  void declare_and_get_params();

  // MAVROS subscriptions
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_state(const mavros_msgs::msg::State::SharedPtr msg);
  void on_traj_desired(const mavros_msgs::msg::Trajectory::SharedPtr msg);
  void on_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg);

  // Optional goal input (MarkerArray) - keep for parity with ROS1
  void on_goal_marker(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  // Pointclouds
  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t index);

  // Main loop
  void cmdloop();

  // Parameters
  std::vector<std::string> pointcloud_topics_;
  std::string frame_fcu_{"fcu"};
  std::string frame_local_{"local_origin"};
  bool accept_goal_input_topic_{false};

  // Subscriptions/pubs
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr goal_marker_sub_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subs_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pos_pub_;
  rclcpp::Publisher<mavros_msgs::msg::Trajectory>::SharedPtr traj_generated_pub_;

  rclcpp::TimerBase::SharedPtr cmdloop_timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  mutable std::mutex state_mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  bool pose_received_{false};

  // TODO: connect PX4 params (e.g. via a separate `avoidance_ros2::Px4ParamBridge` component)
  // and feed them into the ported planner core.
  avoidance_ros2::ModelParameters px4_params_{};
};

}  // namespace local_planner_ros2
