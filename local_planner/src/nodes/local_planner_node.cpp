#include "local_planner/local_planner_node.h"

#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "avoidance/common.h"

using namespace std::chrono_literals;

namespace avoidance {

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("local_planner", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  declareAndLoadParams();
  setupPublishers();
  setupSubscriptions();
  setupTimer();

  planner_.setDefaultPx4Parameters();
  planner_.setParams(planner_params_);
  waypoint_generator_.setSmoothingSpeed(static_cast<float>(smoothing_speed_xy_), static_cast<float>(smoothing_speed_z_));
  
  // Initialize visualization
  if (enable_visualization_) {
    visualization_.initializePublishers(this);
    RCLCPP_INFO(this->get_logger(), "Visualization enabled");
  }
}

void LocalPlannerNode::declareAndLoadParams() {
  // Note: use_sim_time is automatically declared by rclcpp::Node, do not redeclare
  this->declare_parameter<double>("planner_rate_hz", planner_rate_hz_);
  this->declare_parameter<std::string>("pose_topic", pose_topic_);
  this->declare_parameter<std::string>("velocity_topic", velocity_topic_);
  this->declare_parameter<std::string>("trajectory_topic", trajectory_topic_);
  this->declare_parameter<std::string>("setpoint_topic", setpoint_topic_);
  this->declare_parameter<std::string>("obstacle_scan_topic", obstacle_scan_topic_);
  this->declare_parameter<std::vector<std::string>>("pointcloud_topics", pointcloud_topics_);
  this->declare_parameter<std::string>("target_cloud_frame", target_cloud_frame_);

  this->declare_parameter<bool>("use_trajectory_goal", use_trajectory_goal_);
  this->declare_parameter<double>("goal_x", goal_x_);
  this->declare_parameter<double>("goal_y", goal_y_);
  this->declare_parameter<double>("goal_z", goal_z_);

  this->declare_parameter<double>("smoothing_speed_xy", smoothing_speed_xy_);
  this->declare_parameter<double>("smoothing_speed_z", smoothing_speed_z_);

  // Planner parameters (former dynamic_reconfigure)
  this->declare_parameter<double>("max_sensor_range", planner_params_.max_sensor_range);
  this->declare_parameter<double>("min_sensor_range", planner_params_.min_sensor_range);
  this->declare_parameter<double>("pitch_cost_param", planner_params_.pitch_cost_param);
  this->declare_parameter<double>("yaw_cost_param", planner_params_.yaw_cost_param);
  this->declare_parameter<double>("velocity_cost_param", planner_params_.velocity_cost_param);
  this->declare_parameter<double>("obstacle_cost_param", planner_params_.obstacle_cost_param);
  this->declare_parameter<double>("tree_heuristic_weight", planner_params_.tree_heuristic_weight);
  this->declare_parameter<double>("timeout_startup", planner_params_.timeout_startup);
  this->declare_parameter<double>("timeout_critical", planner_params_.timeout_critical);
  this->declare_parameter<double>("timeout_termination", planner_params_.timeout_termination);
  this->declare_parameter<double>("max_point_age_s", planner_params_.max_point_age_s);
  this->declare_parameter<int>("min_num_points_per_cell", planner_params_.min_num_points_per_cell);
  this->declare_parameter<double>("smoothing_margin_degrees", planner_params_.smoothing_margin_degrees);
  this->declare_parameter<int>("children_per_node", planner_params_.children_per_node);
  this->declare_parameter<int>("n_expanded_nodes", planner_params_.n_expanded_nodes);
  this->declare_parameter<double>("tree_node_distance", planner_params_.tree_node_distance);
  this->declare_parameter<double>("camera_yaw_offset_deg", planner_params_.camera_yaw_offset_deg);
  this->declare_parameter<bool>("enable_visualization", enable_visualization_);

  // Fetch
  planner_rate_hz_ = this->get_parameter("planner_rate_hz").as_double();
  pose_topic_ = this->get_parameter("pose_topic").as_string();
  velocity_topic_ = this->get_parameter("velocity_topic").as_string();
  trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();
  setpoint_topic_ = this->get_parameter("setpoint_topic").as_string();
  obstacle_scan_topic_ = this->get_parameter("obstacle_scan_topic").as_string();
  pointcloud_topics_ = this->get_parameter("pointcloud_topics").as_string_array();
  target_cloud_frame_ = this->get_parameter("target_cloud_frame").as_string();

  use_trajectory_goal_ = this->get_parameter("use_trajectory_goal").as_bool();
  goal_x_ = this->get_parameter("goal_x").as_double();
  goal_y_ = this->get_parameter("goal_y").as_double();
  goal_z_ = this->get_parameter("goal_z").as_double();

  smoothing_speed_xy_ = this->get_parameter("smoothing_speed_xy").as_double();
  smoothing_speed_z_ = this->get_parameter("smoothing_speed_z").as_double();

  planner_params_.max_sensor_range = static_cast<float>(this->get_parameter("max_sensor_range").as_double());
  planner_params_.min_sensor_range = static_cast<float>(this->get_parameter("min_sensor_range").as_double());
  planner_params_.pitch_cost_param = static_cast<float>(this->get_parameter("pitch_cost_param").as_double());
  planner_params_.yaw_cost_param = static_cast<float>(this->get_parameter("yaw_cost_param").as_double());
  planner_params_.velocity_cost_param = static_cast<float>(this->get_parameter("velocity_cost_param").as_double());
  planner_params_.obstacle_cost_param = static_cast<float>(this->get_parameter("obstacle_cost_param").as_double());
  planner_params_.tree_heuristic_weight = static_cast<float>(this->get_parameter("tree_heuristic_weight").as_double());
  planner_params_.timeout_startup = this->get_parameter("timeout_startup").as_double();
  planner_params_.timeout_critical = this->get_parameter("timeout_critical").as_double();
  planner_params_.timeout_termination = this->get_parameter("timeout_termination").as_double();
  planner_params_.max_point_age_s = static_cast<float>(this->get_parameter("max_point_age_s").as_double());
  planner_params_.min_num_points_per_cell = this->get_parameter("min_num_points_per_cell").as_int();
  planner_params_.smoothing_margin_degrees = static_cast<float>(this->get_parameter("smoothing_margin_degrees").as_double());
  planner_params_.children_per_node = this->get_parameter("children_per_node").as_int();
  planner_params_.n_expanded_nodes = this->get_parameter("n_expanded_nodes").as_int();
  planner_params_.tree_node_distance = static_cast<float>(this->get_parameter("tree_node_distance").as_double());
  planner_params_.camera_yaw_offset_deg = static_cast<float>(this->get_parameter("camera_yaw_offset_deg").as_double());
  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();

  goal_ = Eigen::Vector3f(static_cast<float>(goal_x_), static_cast<float>(goal_y_), static_cast<float>(goal_z_));
  prev_goal_ = goal_;

  latest_clouds_.resize(pointcloud_topics_.size());
}

void LocalPlannerNode::setupPublishers() {
  setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(setpoint_topic_, rclcpp::QoS(10));
  obstacle_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(obstacle_scan_topic_, rclcpp::QoS(10));
  companion_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>(
      "/mavros/companion_process/status", rclcpp::QoS(10));
  trajectory_generated_pub_ = this->create_publisher<mavros_msgs::msg::Trajectory>(
      "/mavros/trajectory/generated", rclcpp::QoS(10));
}

void LocalPlannerNode::setupSubscriptions() {
  // MAVROS2 uses different QoS for different topics:
  // - Pose/velocity topics: BEST_EFFORT + VOLATILE
  // - Trajectory topics: RELIABLE + VOLATILE
  // - State topic: RELIABLE + TRANSIENT_LOCAL (latched)
  auto sensor_qos = rclcpp::QoS(10).best_effort();
  auto reliable_qos = rclcpp::QoS(10).reliable();
  auto state_qos = rclcpp::QoS(10).reliable().transient_local();

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, sensor_qos, std::bind(&LocalPlannerNode::poseCallback, this, std::placeholders::_1));

  vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_topic_, sensor_qos, std::bind(&LocalPlannerNode::velocityCallback, this, std::placeholders::_1));

  traj_sub_ = this->create_subscription<mavros_msgs::msg::Trajectory>(
      trajectory_topic_, reliable_qos, std::bind(&LocalPlannerNode::trajectoryCallback, this, std::placeholders::_1));

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", state_qos, std::bind(&LocalPlannerNode::stateCallback, this, std::placeholders::_1));

  cloud_subs_.clear();
  cloud_subs_.reserve(pointcloud_topics_.size());
  for (size_t i = 0; i < pointcloud_topics_.size(); ++i) {
    cloud_subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topics_[i], rclcpp::SensorDataQoS(),
        [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloudCallback(i, msg); }));
  }
}

void LocalPlannerNode::setupTimer() {
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(0.1, planner_rate_hz_)));
  timer_ = this->create_wall_timer(period, std::bind(&LocalPlannerNode::onTimer, this));

  // Heartbeat timer at 1Hz to tell PX4 avoidance system is active
  heartbeat_timer_ = this->create_wall_timer(1s, std::bind(&LocalPlannerNode::publishCompanionStatus, this));
}

void LocalPlannerNode::publishCompanionStatus() {
  mavros_msgs::msg::CompanionProcessStatus status;
  // Use system time for MAVLink compatibility
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  status.header.stamp = system_clock.now();
  status.header.frame_id = "base_link";
  status.component = mavros_msgs::msg::CompanionProcessStatus::MAV_COMP_ID_OBSTACLE_AVOIDANCE;
  status.state = mavros_msgs::msg::CompanionProcessStatus::MAV_STATE_ACTIVE;
  companion_status_pub_->publish(status);
}

void LocalPlannerNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  position_ = toEigen(msg->pose.position);
  orientation_ = toEigen(msg->pose.orientation);
  have_pose_ = true;
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  velocity_ = toEigen(msg->twist.linear);
  have_velocity_ = true;
}

void LocalPlannerNode::trajectoryCallback(const mavros_msgs::msg::Trajectory::SharedPtr msg) {
  if (!use_trajectory_goal_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // Store the complete trajectory message for generating response
  latest_trajectory_msg_ = *msg;
  have_trajectory_ = true;

  // MAVROS Trajectory typically uses point_1 as the active setpoint.
  // Keep previous goal for off-track logic.
  if (std::isfinite(goal_.x()) && std::isfinite(goal_.y())) {
    prev_goal_ = goal_;
  }

  goal_ = Eigen::Vector3f(static_cast<float>(msg->point_1.position.x), static_cast<float>(msg->point_1.position.y),
                         static_cast<float>(msg->point_1.position.z));
  desired_vel_ = Eigen::Vector3f(static_cast<float>(msg->point_1.velocity.x), static_cast<float>(msg->point_1.velocity.y),
                                 static_cast<float>(msg->point_1.velocity.z));

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Trajectory received: goal=[%.2f, %.2f, %.2f]",
                       goal_.x(), goal_.y(), goal_.z());

  // Basic land/takeoff inference if available
  if (msg->command.size() > 1) {
    is_land_waypoint_ = (msg->command[1] == static_cast<uint16_t>(MavCommand::MAV_CMD_NAV_LAND));
    is_takeoff_waypoint_ = (msg->command[1] == static_cast<uint16_t>(MavCommand::MAV_CMD_NAV_TAKEOFF));
  }
}

void LocalPlannerNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  is_airborne_ = msg->armed;
  loiter_ = (msg->mode == "AUTO.LOITER");
  nav_state_ = (msg->mode.find("OFFBOARD") != std::string::npos) ? NavigationState::offboard : NavigationState::mission;
}

void LocalPlannerNode::cloudCallback(size_t index, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (index >= latest_clouds_.size()) {
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_clouds_[index] = *msg;
  have_cloud_ = true;
}

void LocalPlannerNode::onTimer() {
  // Snapshot inputs
  std::vector<sensor_msgs::msg::PointCloud2> clouds;
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf orientation;
  Eigen::Vector3f goal;
  Eigen::Vector3f prev_goal;
  Eigen::Vector3f desired_vel;
  bool loiter;
  bool is_airborne;
  bool is_land_waypoint;
  bool is_takeoff_waypoint;
  NavigationState nav_state;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!have_pose_ || !have_cloud_) {
      return;
    }
    clouds = latest_clouds_;
    position = position_;
    velocity = have_velocity_ ? velocity_ : Eigen::Vector3f::Zero();
    orientation = orientation_;
    goal = goal_;
    prev_goal = prev_goal_;
    desired_vel = desired_vel_;
    loiter = loiter_;
    is_airborne = is_airborne_;
    is_land_waypoint = is_land_waypoint_;
    is_takeoff_waypoint = is_takeoff_waypoint_;
    nav_state = nav_state_;
  }

  if (!std::isfinite(goal.x()) || !std::isfinite(goal.y()) || !std::isfinite(goal.z())) {
    // Fall back to static goal params
    goal = Eigen::Vector3f(static_cast<float>(goal_x_), static_cast<float>(goal_y_), static_cast<float>(goal_z_));
    prev_goal = goal;
  }

  // Convert and feed clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>> pcl_clouds;
  pcl_clouds.resize(clouds.size());
  std::vector<FOV> fovs;
  fovs.resize(clouds.size());

  for (size_t i = 0; i < clouds.size(); ++i) {
    if (clouds[i].data.empty()) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 cloud_in_target;
    try {
      // Transform cloud into desired planning frame
      if (!target_cloud_frame_.empty() && clouds[i].header.frame_id != target_cloud_frame_) {
        // Use Time(0) to get the latest available transform instead of trying to match timestamps
        // This is necessary because Gazebo sim time and system time may differ significantly
        const auto tf = tf_buffer_.lookupTransform(target_cloud_frame_, clouds[i].header.frame_id,
                                                   rclcpp::Time(0), 100ms);
        tf2::doTransform(clouds[i], cloud_in_target, tf);
      } else {
        cloud_in_target = clouds[i];
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF transform failed for cloud[%zu]: %s", i, e.what());
      continue;
    }

    pcl::fromROSMsg(cloud_in_target, pcl_clouds[i]);
    auto maxima = removeNaNAndGetMaxima(pcl_clouds[i]);
    updateFOVFromMaxima(fovs[i], maxima);
  }

  planner_.original_cloud_vector_ = pcl_clouds;
  for (size_t i = 0; i < fovs.size(); ++i) {
    planner_.setFOV(static_cast<int>(i), fovs[i]);
  }

  planner_.setState(position, velocity, orientation);
  planner_.setPreviousGoal(prev_goal);
  planner_.setGoal(goal);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Goal: [%.2f, %.2f, %.2f], Position: [%.2f, %.2f, %.2f]",
                       goal.x(), goal.y(), goal.z(), position.x(), position.y(), position.z());

  planner_.runPlanner();
  const auto planner_out = planner_.getAvoidanceOutput();

  waypoint_generator_.setSmoothingSpeed(static_cast<float>(smoothing_speed_xy_), static_cast<float>(smoothing_speed_z_));
  waypoint_generator_.setPlannerInfo(planner_out);
  waypoint_generator_.updateState(position, orientation, goal, prev_goal, velocity, loiter, is_airborne, nav_state,
                                  is_land_waypoint, is_takeoff_waypoint, desired_vel);

  const auto wps = waypoint_generator_.getWaypoints();

  // Publish setpoint for OFFBOARD mode
  geometry_msgs::msg::PoseStamped sp;
  sp.header.stamp = this->now();
  sp.header.frame_id = "local_origin";
  sp.pose.position = toPoint(wps.position_wp);
  sp.pose.orientation = toQuaternion(wps.orientation_wp);
  setpoint_pub_->publish(sp);

  // Publish trajectory/generated for Mission mode avoidance
  // This is what PX4 uses when COM_OBS_AVOID is enabled in auto modes
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    mavros_msgs::msg::Trajectory traj_msg;
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    traj_msg.header.stamp = system_clock.now();
    traj_msg.header.frame_id = "local_origin";
    traj_msg.type = mavros_msgs::msg::Trajectory::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;

    // Set avoidance waypoint in point_1 (modified setpoint from avoidance)
    traj_msg.point_1.position.x = static_cast<double>(wps.position_wp.x());
    traj_msg.point_1.position.y = static_cast<double>(wps.position_wp.y());
    traj_msg.point_1.position.z = static_cast<double>(wps.position_wp.z());
    traj_msg.point_1.velocity.x = static_cast<double>(wps.linear_velocity_wp.x());
    traj_msg.point_1.velocity.y = static_cast<double>(wps.linear_velocity_wp.y());
    traj_msg.point_1.velocity.z = static_cast<double>(wps.linear_velocity_wp.z());
    
    // Extract yaw from quaternion orientation
    const auto& q = wps.orientation_wp;
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    traj_msg.point_1.yaw = yaw;
    traj_msg.point_1.yaw_rate = NAN;  // Let PX4 control yaw rate
    
    // Copy the original desired trajectory points 2-5 and commands from PX4
    // This is important for PX4 to understand the mission context
    if (have_trajectory_) {
      traj_msg.point_2 = latest_trajectory_msg_.point_2;
      traj_msg.point_3 = latest_trajectory_msg_.point_3;
      traj_msg.point_4 = latest_trajectory_msg_.point_4;
      traj_msg.point_5 = latest_trajectory_msg_.point_5;
      
      // Copy point_valid and command arrays
      for (int i = 0; i < 5; ++i) {
        traj_msg.point_valid[i] = latest_trajectory_msg_.point_valid[i];
        traj_msg.command[i] = latest_trajectory_msg_.command[i];
        traj_msg.time_horizon[i] = latest_trajectory_msg_.time_horizon[i];
      }
    }
    
    // Always mark point_1 as valid (our avoidance output)
    traj_msg.point_valid[0] = true;

    trajectory_generated_pub_->publish(traj_msg);
  }

  sensor_msgs::msg::LaserScan scan;
  planner_.getObstacleDistanceData(scan);
  // Use system time for obstacle data (PX4 expects wall clock time)
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  scan.header.stamp = system_clock.now();
  obstacle_scan_pub_->publish(scan);

  // Visualization
  if (enable_visualization_) {
    visualization_.visualizePlannerData(planner_, wps.smoothed_goto_position, wps.adapted_goto_position, position, orientation);
  }
}

}  // namespace avoidance
