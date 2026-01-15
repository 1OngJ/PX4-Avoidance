#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mavros_msgs/msg/companion_process_status.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/trajectory.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/local_planner.h"
#include "local_planner/waypoint_generator.h"

namespace avoidance {

class LocalPlannerNode final : public rclcpp::Node {
 public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void declareAndLoadParams();
  void setupSubscriptions();
  void setupPublishers();
  void setupTimer();
  void onTimer();
  void publishCompanionStatus();

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void trajectoryCallback(const mavros_msgs::msg::Trajectory::SharedPtr msg);
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void cloudCallback(size_t index, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Parameters
  double planner_rate_hz_{10.0};
  std::string pose_topic_{"/mavros/local_position/pose"};
  std::string velocity_topic_{"/mavros/local_position/velocity_local"};
  std::string trajectory_topic_{"/mavros/trajectory/desired"};
  std::string setpoint_topic_{"/mavros/setpoint_position/local"};
  std::string obstacle_scan_topic_{"/local_planner/obstacle_scan"};
  std::vector<std::string> pointcloud_topics_{"/cloud_in"};
  std::string target_cloud_frame_{"fcu"};

  bool use_trajectory_goal_{true};
  double goal_x_{17.0};
  double goal_y_{15.0};
  double goal_z_{3.5};

  double smoothing_speed_xy_{10.0};
  double smoothing_speed_z_{3.0};

  // Planner params
  LocalPlanner::Params planner_params_;

  // State
  std::mutex data_mutex_;
  bool have_pose_{false};
  bool have_velocity_{false};
  bool have_cloud_{false};

  Eigen::Vector3f position_{Eigen::Vector3f::Constant(NAN)};
  Eigen::Vector3f velocity_{Eigen::Vector3f::Constant(NAN)};
  Eigen::Quaternionf orientation_{Eigen::Quaternionf::Identity()};

  Eigen::Vector3f goal_{Eigen::Vector3f::Constant(NAN)};
  Eigen::Vector3f prev_goal_{Eigen::Vector3f::Constant(NAN)};
  Eigen::Vector3f desired_vel_{Eigen::Vector3f::Constant(NAN)};

  // Store latest trajectory message for generating response
  mavros_msgs::msg::Trajectory latest_trajectory_msg_;
  bool have_trajectory_{false};

  bool loiter_{false};
  bool is_airborne_{false};
  bool is_land_waypoint_{false};
  bool is_takeoff_waypoint_{false};
  NavigationState nav_state_{NavigationState::none};

  std::vector<sensor_msgs::msg::PointCloud2> latest_clouds_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Core
  LocalPlanner planner_;
  WaypointGenerator waypoint_generator_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<mavros_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subs_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_scan_pub_;
  rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr companion_status_pub_;
  rclcpp::Publisher<mavros_msgs::msg::Trajectory>::SharedPtr trajectory_generated_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace avoidance
