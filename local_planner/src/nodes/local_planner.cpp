#include "local_planner/local_planner.h"

#include "avoidance/common.h"
#include "local_planner/star_planner.h"
#include "local_planner/tree_node.h"

#include <cmath>

namespace avoidance {

LocalPlanner::LocalPlanner()
    : last_path_time_(steady_clock_.now()),
      last_pointcloud_process_time_(steady_clock_.now()),
      star_planner_(new StarPlanner()) {}

LocalPlanner::~LocalPlanner() {}

// update UAV pose
void LocalPlanner::setState(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, const Eigen::Quaternionf& q) {
  position_ = pos;
  velocity_ = vel;
  yaw_fcu_frame_deg_ = getYawFromQuaternion(q);
  pitch_fcu_frame_deg_ = getPitchFromQuaternion(q);
  star_planner_->setPose(position_, velocity_);
}

void LocalPlanner::setParams(const Params& params) {
  max_sensor_range_ = params.max_sensor_range;
  min_sensor_range_ = params.min_sensor_range;
  max_point_age_s_ = params.max_point_age_s;
  forward_camera_index_ = params.forward_camera_index;
  non_forward_initial_age_s_ = params.non_forward_initial_age_s;
  min_num_points_per_cell_ = params.min_num_points_per_cell;
  timeout_startup_ = params.timeout_startup;
  timeout_critical_ = params.timeout_critical;
  timeout_termination_ = params.timeout_termination;
  smoothing_margin_degrees_ = params.smoothing_margin_degrees;
  camera_yaw_offset_deg_ = params.camera_yaw_offset_deg;

  children_per_node_ = params.children_per_node;
  n_expanded_nodes_ = params.n_expanded_nodes;

  cost_params_.pitch_cost_param = params.pitch_cost_param;
  cost_params_.yaw_cost_param = params.yaw_cost_param;
  cost_params_.velocity_cost_param = params.velocity_cost_param;
  cost_params_.obstacle_cost_param = params.obstacle_cost_param;

  // Pass tree search parameters to StarPlanner
  star_planner_->setTreeParams(
      params.children_per_node,
      params.n_expanded_nodes,
      params.tree_node_distance,
      params.tree_heuristic_weight,
      params.max_sensor_range,
      params.min_sensor_range,
      params.smoothing_margin_degrees);

  RCLCPP_INFO(logger_, "\033[0;35m[OA] Params updated: children_per_node=%d, n_expanded_nodes=%d, "
              "tree_heuristic_weight=%.1f, pitch_cost=%.1f, yaw_cost=%.1f, velocity_cost=%.1f\033[0m",
              params.children_per_node, params.n_expanded_nodes, params.tree_heuristic_weight,
              params.pitch_cost_param, params.yaw_cost_param, params.velocity_cost_param);
}

void LocalPlanner::setGoal(const Eigen::Vector3f& goal) {
  goal_ = goal;

  RCLCPP_INFO(logger_, "===== Set Goal ======: [%f, %f, %f].", goal_.x(), goal_.y(), goal_.z());
  applyGoal();
}
void LocalPlanner::setPreviousGoal(const Eigen::Vector3f& prev_goal) { prev_goal_ = prev_goal; }

void LocalPlanner::setFOV(int i, const FOV& fov) {
  if (i < fov_fcu_frame_.size()) {
    fov_fcu_frame_[i] = fov;
  } else {
    fov_fcu_frame_.push_back(fov);
  }
}

Eigen::Vector3f LocalPlanner::getGoal() const { return goal_; }

void LocalPlanner::applyGoal() { star_planner_->setGoal(goal_); }

void LocalPlanner::runPlanner() {
  RCLCPP_INFO(logger_, "\033[1;35m[OA] Planning started, using %i cameras\n \033[0m",
              static_cast<int>(original_cloud_vector_.size()));

  const auto now_steady = steady_clock_.now();
  float elapsed_since_last_processing = static_cast<float>((now_steady - last_pointcloud_process_time_).seconds());
  processPointcloud(final_cloud_, original_cloud_vector_, fov_fcu_frame_, yaw_fcu_frame_deg_, pitch_fcu_frame_deg_,
                    position_, min_sensor_range_, max_sensor_range_, max_point_age_s_, elapsed_since_last_processing,
                    min_num_points_per_cell_, forward_camera_index_, non_forward_initial_age_s_);
  last_pointcloud_process_time_ = now_steady;

  determineStrategy();
}

void LocalPlanner::create2DObstacleRepresentation(const bool send_to_fcu) {
  // construct histogram if it is needed
  // or if it is required by the FCU
  Histogram new_histogram = Histogram(ALPHA_RES);
  to_fcu_histogram_.setZero();
  generateNewHistogram(new_histogram, final_cloud_, position_);

  if (send_to_fcu) {
    compressHistogramElevation(to_fcu_histogram_, new_histogram, position_);
    updateObstacleDistanceMsg(to_fcu_histogram_);
  }
  polar_histogram_ = new_histogram;

  // generate histogram image for logging
  generateHistogramImage(polar_histogram_);
}

void LocalPlanner::generateHistogramImage(Histogram& histogram) {
  histogram_image_data_.clear();
  histogram_image_data_.reserve(GRID_LENGTH_E * GRID_LENGTH_Z);

  // fill image data
  for (int e = GRID_LENGTH_E - 1; e >= 0; e--) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      float dist = histogram.get_dist(e, z);
      float depth_val = dist > 0.01f ? 255.f - 255.f * dist / max_sensor_range_ : 0.f;
      histogram_image_data_.push_back((int)std::max(0.0f, std::min(255.f, depth_val)));
    }
  }
}

void LocalPlanner::determineStrategy() {
  // clear cost image
  cost_image_data_.clear();
  cost_image_data_.resize(3 * GRID_LENGTH_E * GRID_LENGTH_Z, 0);

  create2DObstacleRepresentation(px4_.param_cp_dist > 0.f);

  // calculate the vehicle projected position on the line between the previous and current goal
  Eigen::Vector2f u_prev_to_goal = (goal_ - prev_goal_).head<2>().normalized();
  Eigen::Vector2f prev_to_pos = (position_ - prev_goal_).head<2>();
  closest_pt_.head<2>() = prev_goal_.head<2>() + (u_prev_to_goal * u_prev_to_goal.dot(prev_to_pos));
  closest_pt_.z() = goal_.z();

  // if the vehicle is less than the cruise speed away from the line or if prev goal is the same as goal,
  // set the projection point to the goal such that the cost function doesn't pull the vehicle towards the line
  if ((position_ - closest_pt_).head<2>().norm() < px4_.param_mpc_xy_cruise ||
      (goal_ - prev_goal_).head<2>().norm() < 0.001f) {
    closest_pt_ = goal_;
  }

  if (!polar_histogram_.isEmpty()) {
    getCostMatrix(polar_histogram_, goal_, position_, velocity_, cost_params_, smoothing_margin_degrees_, closest_pt_,
                  max_sensor_range_, min_sensor_range_, cost_matrix_, cost_image_data_);

    star_planner_->setParams(cost_params_);
    star_planner_->setPointcloud(final_cloud_);
    star_planner_->setClosestPointOnLine(closest_pt_);

    // build search tree
    star_planner_->buildLookAheadTree();
    last_path_time_ = steady_clock_.now();
  }
}

void LocalPlanner::updateObstacleDistanceMsg(Histogram hist) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  msg.header.frame_id = "base_link";
  msg.angle_min = 0.0f;
  msg.angle_max = 2.0f * static_cast<float>(M_PI)
                  - static_cast<float>(ALPHA_RES) * static_cast<float>(M_PI) / 180.0f;
  msg.angle_increment = static_cast<float>(ALPHA_RES) * static_cast<float>(M_PI) / 180.0f;
  msg.range_min = min_sensor_range_;
  msg.range_max = max_sensor_range_;
  msg.ranges.resize(GRID_LENGTH_Z);

  // ---- Coordinate convention ----
  // MAVROS obstacle plugin default: mav_frame = "GLOBAL"
  //   → MAVLink OBSTACLE_DISTANCE.frame = MAV_FRAME_GLOBAL
  //   → PX4 interprets angles as NED heading: 0°=North, CW
  //
  // Histogram azimuth: atan2(east, north) → 0°=North, 90°=East, CW
  //   → SAME convention as NED heading!
  //
  // So the mapping is trivial:
  //   ranges[i] = distance at NED heading (i * ALPHA_RES)°
  //   hist_az   = i * ALPHA_RES  (direct 1:1 mapping)

  for (int i = 0; i < GRID_LENGTH_Z; ++i) {
    // NED heading angle for this bin (0°=North, CW)
    // Same convention as histogram azimuth → direct mapping
    float hist_az_deg = static_cast<float>(i * ALPHA_RES);

    // Wrap to [-180, 180) for histogram index calculation
    while (hist_az_deg > 180.0f) hist_az_deg -= 360.0f;
    while (hist_az_deg <= -180.0f) hist_az_deg += 360.0f;

    int z_idx = static_cast<int>(std::floor(hist_az_deg / static_cast<float>(ALPHA_RES)
                                            + static_cast<float>(GRID_LENGTH_Z) / 2.0f));
    z_idx = std::max(0, std::min(z_idx, GRID_LENGTH_Z - 1));

    float dist = hist.get_dist(0, z_idx);

    if (histogramIndexYawInsideFOV(fov_fcu_frame_, z_idx, position_, yaw_fcu_frame_deg_)) {
      msg.ranges[i] = dist > min_sensor_range_ ? dist : max_sensor_range_ + 0.01f;
    } else {
      msg.ranges[i] = max_sensor_range_ + 1.00f;
    }
  }

  distance_data_ = msg;
}

void LocalPlanner::updateObstacleDistanceMsg() {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  msg.header.frame_id = "map";
  msg.angle_increment = static_cast<float>(ALPHA_RES) * M_PI / 180.0f;
  msg.range_min = min_sensor_range_;
  msg.range_max = max_sensor_range_;

  distance_data_ = msg;
}

Eigen::Vector3f LocalPlanner::getPosition() const { return position_; }

const pcl::PointCloud<pcl::PointXYZI>& LocalPlanner::getPointcloud() const { return final_cloud_; }

void LocalPlanner::setDefaultPx4Parameters() {
  px4_.param_mpc_auto_mode = 1;
  px4_.param_mpc_jerk_min = 8.f;
  px4_.param_mpc_jerk_max = 20.f;
  px4_.param_mpc_acc_up_max = 10.f;
  px4_.param_mpc_z_vel_max_up = 3.f;
  px4_.param_mpc_acc_down_max = 10.f;
  px4_.param_mpc_acc_hor = 5.f;
  px4_.param_mpc_xy_cruise = 10.f;
  px4_.param_mpc_tko_speed = 1.f;
  px4_.param_mpc_land_speed = 0.7f;
  px4_.param_cp_dist = 4.f;
}

void LocalPlanner::getTree(std::vector<TreeNode>& tree, std::vector<int>& closed_set,
                           std::vector<Eigen::Vector3f>& path_node_positions) const {
  tree = star_planner_->tree_;
  closed_set = star_planner_->closed_set_;
  path_node_positions = star_planner_->path_node_positions_;
}

void LocalPlanner::getObstacleDistanceData(sensor_msgs::msg::LaserScan& obstacle_distance) {
  obstacle_distance = distance_data_;
}

avoidanceOutput LocalPlanner::getAvoidanceOutput() const {
  avoidanceOutput out;

  // calculate maximum speed given the sensor range and vehicle parameters
  // quadratic solve of 0 = u^2 + 2as, with s = u * |a/j| + r
  // u = initial velocity, a = max acceleration
  // s = stopping distance under constant acceleration
  // j = maximum jerk, r = maximum range sensor distance
  float accel_ramp_time = px4_.param_mpc_acc_hor / px4_.param_mpc_jerk_max;
  float a = 1;
  float b = 2 * px4_.param_mpc_acc_hor * accel_ramp_time;
  float c = 2 * -px4_.param_mpc_acc_hor * max_sensor_range_;
  float limited_speed = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);

  float speed = std::isfinite(mission_item_speed_) ? mission_item_speed_ : px4_.param_mpc_xy_cruise;
  float max_speed = std::min(speed, limited_speed);

  out.cruise_velocity = max_speed;
  out.last_path_time = last_path_time_;

  out.path_node_positions = star_planner_->path_node_positions_;
  return out;
}
}
