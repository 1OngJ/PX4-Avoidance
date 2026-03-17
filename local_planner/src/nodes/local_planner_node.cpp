#include "local_planner/local_planner_node.h"

#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "avoidance/common.h"
#include "local_planner/planner_functions.h"

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

  RCLCPP_INFO(this->get_logger(), "LocalPlannerNode initialized");
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

  this->declare_parameter<bool>("enable_visualization", enable_visualization_);

  // Planner parameters
  this->declare_parameter<double>("max_sensor_range", planner_params_.max_sensor_range);
  this->declare_parameter<double>("min_sensor_range", planner_params_.min_sensor_range);
  this->declare_parameter<double>("pitch_cost_param", planner_params_.pitch_cost_param);
  this->declare_parameter<double>("yaw_cost_param", planner_params_.yaw_cost_param);
  this->declare_parameter<double>("velocity_cost_param", planner_params_.velocity_cost_param);
  this->declare_parameter<double>("obstacle_cost_param", planner_params_.obstacle_cost_param);
  this->declare_parameter<double>("tree_heuristic_weight", planner_params_.tree_heuristic_weight);
  this->declare_parameter<int>("n_expanded_nodes", planner_params_.n_expanded_nodes);
  this->declare_parameter<double>("max_point_age_s", planner_params_.max_point_age_s);
  this->declare_parameter<int>("min_num_points_per_cell", planner_params_.min_num_points_per_cell);
  this->declare_parameter<double>("smoothing_margin_degrees", planner_params_.smoothing_margin_degrees);
  this->declare_parameter<int>("children_per_node", planner_params_.children_per_node);
  this->declare_parameter<double>("tree_node_distance", planner_params_.tree_node_distance);
  this->declare_parameter<double>("camera_yaw_offset_deg", planner_params_.camera_yaw_offset_deg);
  this->declare_parameter<int>("forward_camera_index", planner_params_.forward_camera_index);
  this->declare_parameter<double>("non_forward_initial_age_s", planner_params_.non_forward_initial_age_s);

  // Load parameters
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

  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();

  // Load planner parameters
  planner_params_.max_sensor_range = static_cast<float>(this->get_parameter("max_sensor_range").as_double());
  planner_params_.min_sensor_range = static_cast<float>(this->get_parameter("min_sensor_range").as_double());
  planner_params_.pitch_cost_param = static_cast<float>(this->get_parameter("pitch_cost_param").as_double());
  planner_params_.yaw_cost_param = static_cast<float>(this->get_parameter("yaw_cost_param").as_double());
  planner_params_.velocity_cost_param = static_cast<float>(this->get_parameter("velocity_cost_param").as_double());
  planner_params_.obstacle_cost_param = static_cast<float>(this->get_parameter("obstacle_cost_param").as_double());
  planner_params_.tree_heuristic_weight = static_cast<float>(this->get_parameter("tree_heuristic_weight").as_double());
  planner_params_.n_expanded_nodes = this->get_parameter("n_expanded_nodes").as_int();
  planner_params_.max_point_age_s = static_cast<float>(this->get_parameter("max_point_age_s").as_double());
  planner_params_.min_num_points_per_cell = this->get_parameter("min_num_points_per_cell").as_int();
  planner_params_.smoothing_margin_degrees = static_cast<float>(this->get_parameter("smoothing_margin_degrees").as_double());
  planner_params_.children_per_node = this->get_parameter("children_per_node").as_int();
  planner_params_.tree_node_distance = static_cast<float>(this->get_parameter("tree_node_distance").as_double());
  planner_params_.camera_yaw_offset_deg = static_cast<float>(this->get_parameter("camera_yaw_offset_deg").as_double());
  planner_params_.forward_camera_index = this->get_parameter("forward_camera_index").as_int();
  planner_params_.non_forward_initial_age_s = static_cast<float>(this->get_parameter("non_forward_initial_age_s").as_double());

  // Initialize goal from parameters
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
      pose_topic_, sensor_qos,
      std::bind(&LocalPlannerNode::poseCallback, this, std::placeholders::_1));

  vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_topic_, sensor_qos,
      std::bind(&LocalPlannerNode::velocityCallback, this, std::placeholders::_1));

  traj_sub_ = this->create_subscription<mavros_msgs::msg::Trajectory>(
      trajectory_topic_, reliable_qos,
      std::bind(&LocalPlannerNode::trajectoryCallback, this, std::placeholders::_1));

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", state_qos,
      std::bind(&LocalPlannerNode::stateCallback, this, std::placeholders::_1));

  // Subscribe to point clouds
  for (size_t i = 0; i < pointcloud_topics_.size(); ++i) {
    auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topics_[i], sensor_qos,
        [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->cloudCallback(i, msg);
        });
    cloud_subs_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud topic: %s", pointcloud_topics_[i].c_str());
  }
}

void LocalPlannerNode::setupTimer() {
  auto period = std::chrono::duration<double>(1.0 / planner_rate_hz_);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalPlannerNode::onTimer, this));

  // Heartbeat timer for companion status (1 Hz)
  heartbeat_timer_ = this->create_wall_timer(
      1s, std::bind(&LocalPlannerNode::publishCompanionStatus, this));
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

  // Update goal from trajectory message
  bool update_goal = false;
  if (msg->point_valid[0]) {
    Eigen::Vector3f new_goal = toEigen(msg->point_1.position);
    // Check if goal has changed significantly
    if (!std::isfinite(goal_.x()) || !std::isfinite(goal_.y()) ||
        (new_goal - goal_).norm() > 0.1f) {
      update_goal = true;
    }
  }

  if (update_goal && msg->point_valid[0]) {
    goal_ = toEigen(msg->point_1.position);
    desired_vel_ = toEigen(msg->point_1.velocity);
    is_land_waypoint_ = (msg->command[0] == static_cast<uint16_t>(MavCommand::MAV_CMD_NAV_LAND));
    is_takeoff_waypoint_ = (msg->command[0] == static_cast<uint16_t>(MavCommand::MAV_CMD_NAV_TAKEOFF));

    RCLCPP_DEBUG(this->get_logger(), "Updated goal from trajectory: [%.2f, %.2f, %.2f]",
                 goal_.x(), goal_.y(), goal_.z());
  }

  // Also check point_2 for mission item info
  if (msg->point_valid[1]) {
    if (msg->command[1] == UINT16_MAX) {
      goal_ = toEigen(msg->point_2.position);
      desired_vel_ << NAN, NAN, NAN;
    }
  }
}

void LocalPlannerNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  is_airborne_ = msg->armed;

  if (msg->mode == "AUTO.MISSION") {
    nav_state_ = NavigationState::mission;
  } else if (msg->mode == "AUTO.TAKEOFF") {
    nav_state_ = NavigationState::auto_takeoff;
  } else if (msg->mode == "AUTO.LAND") {
    nav_state_ = NavigationState::auto_land;
  } else if (msg->mode == "AUTO.RTL") {
    nav_state_ = NavigationState::auto_rtl;
  } else if (msg->mode == "AUTO.RTGS") {
    nav_state_ = NavigationState::auto_rtgs;
  } else if (msg->mode == "AUTO.LOITER") {
    nav_state_ = NavigationState::auto_loiter;
  } else if (msg->mode == "OFFBOARD") {
    nav_state_ = NavigationState::offboard;
  } else {
    nav_state_ = NavigationState::none;
  }
}

void LocalPlannerNode::cloudCallback(size_t index, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (index < latest_clouds_.size()) {
    latest_clouds_[index] = *msg;
    have_cloud_ = true;
  }
}

void LocalPlannerNode::onTimer() {
  // Check if we have required data
  bool have_required_data = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    have_required_data = have_pose_ && have_cloud_;
  }

  if (!have_required_data) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Waiting for pose and point cloud data...");
    return;
  }

  // Copy data under lock
  Eigen::Vector3f position, velocity, goal, prev_goal, desired_vel;
  Eigen::Quaternionf orientation;
  bool loiter, is_airborne, is_land_waypoint, is_takeoff_waypoint;
  NavigationState nav_state;
  std::vector<sensor_msgs::msg::PointCloud2> clouds;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    position = position_;
    velocity = velocity_;
    orientation = orientation_;
    goal = goal_;
    prev_goal = prev_goal_;
    desired_vel = desired_vel_;
    loiter = loiter_;
    is_airborne = is_airborne_;
    is_land_waypoint = is_land_waypoint_;
    is_takeoff_waypoint = is_takeoff_waypoint_;
    nav_state = nav_state_;
    clouds = latest_clouds_;
  }

  // Transform point clouds to local_origin frame
  std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_vector;
  std::vector<FOV> fov_vector;
  for (size_t i = 0; i < clouds.size(); ++i) {
    FOV fov;
    if (clouds[i].data.empty()) {
      cloud_vector.push_back(pcl::PointCloud<pcl::PointXYZ>());
      fov_vector.push_back(fov);
      continue;
    }

    try
    {
      geometry_msgs::msg::TransformStamped fcu_transform;
      try
      {
        fcu_transform = tf_buffer_.lookupTransform(
          "base_link",clouds[i].header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(0.1));

        // Transform cloud to FCU frame for FOV calculation
        sensor_msgs::msg::PointCloud2 cloud_fcu;
        tf2::doTransform(clouds[i], cloud_fcu, fcu_transform);
        
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_fcu;
        pcl::fromROSMsg(cloud_fcu, pcl_cloud_fcu);
        
        // Compute FOV from point cloud maxima
        auto maxima = removeNaNAndGetMaxima(pcl_cloud_fcu);
        updateFOVFromMaxima(fov, maxima);
        
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Could not get FCU transform for FOV: %s", ex.what());
        // Use default FOV for depth camera
        fov = FOV(0.0f, 0.0f, 87.0f, 58.0f);  // OakD-Lite specs: 87° HFOV, 58° VFOV
      }
    
      // Try to get transform to map frame (MAVROS uses "map" as local origin)
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_.lookupTransform(
          target_cloud_frame_, clouds[i].header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(0.1));

      // Transform the point cloud
      sensor_msgs::msg::PointCloud2 transformed_cloud;
      tf2::doTransform(clouds[i], transformed_cloud, transform);

      // Convert to PCL
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(transformed_cloud, pcl_cloud);
      cloud_vector.push_back(pcl_cloud);
      fov_vector.push_back(fov);

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not transform point cloud: %s", ex.what());
      cloud_vector.push_back(pcl::PointCloud<pcl::PointXYZ>());
      fov_vector.push_back(fov);
    }

  }

  // Update planner state
  planner_.setState(position, velocity, orientation);
  planner_.setGoal(goal);
  planner_.setPreviousGoal(prev_goal);
  planner_.original_cloud_vector_ = cloud_vector;

  for(size_t i = 0; i < fov_vector.size(); ++i) {
  planner_.setFOV(static_cast<int>(i), fov_vector[i]);
  waypoint_generator_.setFOV(static_cast<int>(i), fov_vector[i]);
  }

  // Run planner
  planner_.runPlanner();

  // Update waypoint generator
  waypoint_generator_.updateState(position, orientation, goal, prev_goal, velocity,
                                  loiter, is_airborne, nav_state,
                                  is_land_waypoint, is_takeoff_waypoint, desired_vel);
  waypoint_generator_.setPlannerInfo(planner_.getAvoidanceOutput());

  // Get waypoints
  const auto wps = waypoint_generator_.getWaypoints();

  // Publish setpoint for OFFBOARD mode
  geometry_msgs::msg::PoseStamped sp;
  sp.header.stamp = this->now();
  sp.header.frame_id = "map";
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
    traj_msg.header.frame_id = "map";
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

  // Publish obstacle scan data
  sensor_msgs::msg::LaserScan scan;
  planner_.getObstacleDistanceData(scan);
  // Use system time for obstacle data (PX4 expects wall clock time)
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  scan.header.stamp = system_clock.now();
  obstacle_scan_pub_->publish(scan);

  // Visualization
  if (enable_visualization_) {
    visualization_.visualizePlannerData(planner_, wps.position_wp, wps.adapted_goto_position,
                                        position, orientation);
    visualization_.visualizeWaypoints(wps.goto_position, wps.adapted_goto_position, wps.smoothed_goto_position);
    visualization_.publishCurrentSetpoint(toTwist(wps.linear_velocity_wp, wps.angular_velocity_wp),
                                          wps.waypoint_type, position);
  }
}

void LocalPlannerNode::publishCompanionStatus() {
  mavros_msgs::msg::CompanionProcessStatus status_msg;
  status_msg.header.stamp = this->now();
  status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  
  // Set state based on data availability
  if (have_pose_ && have_cloud_) {
    status_msg.state = static_cast<int>(MAV_STATE::MAV_STATE_ACTIVE);
  } else {
    status_msg.state = static_cast<int>(MAV_STATE::MAV_STATE_BOOT);
  }

  companion_status_pub_->publish(status_msg);
}

}  // namespace avoidance
