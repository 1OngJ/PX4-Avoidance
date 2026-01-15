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

  // StarPlanner uses some of these parameters internally
  // (tree heuristic weight, node distance, etc.)
  // We keep the logic centralized here to avoid dynamic_reconfigure.
  // Note: StarPlanner currently only exposes cost params; other values are
  // stored locally and used via children_per_node_/n_expanded_nodes_.
  // tree_heuristic_weight_ is stored in cost_params_ consumer side.
  (void)params.tree_node_distance;
  (void)params.tree_heuristic_weight;

  RCLCPP_DEBUG(logger_, "\033[0;35m[OA] Params updated\033[0m");
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
                    min_num_points_per_cell_);
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
  sensor_msgs::msg::LaserScan msg = {};
  {
    const auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const int64_t now_ns = now.nanoseconds();
    msg.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
  }
  msg.header.frame_id = "base_link";
  
  // PX4 OBSTACLE_DISTANCE format:
  // - 72 sectors, 5° each, covering 360°
  // - Index 0 = forward (0°)
  // - Index increases clockwise (right side: 0->35, left side from rear: 36->71)
  // - Index 0 = 0° (front), Index 18 = 90° (right), Index 36 = 180° (rear), Index 54 = 270° (left)
  const int PX4_SECTORS = 72;
  const float PX4_SECTOR_DEG = 5.0f;
  
  msg.angle_increment = PX4_SECTOR_DEG * M_PI / 180.0f;
  msg.angle_min = 0.0f;  // Start from forward (0°)
  msg.angle_max = (PX4_SECTORS - 1) * msg.angle_increment;  // 355°
  msg.time_increment = 0.0f;
  msg.scan_time = 0.1f;  // 10 Hz
  msg.range_min = min_sensor_range_;
  msg.range_max = max_sensor_range_;
  msg.ranges.reserve(PX4_SECTORS);
  
  const int camera_offset_bins = static_cast<int>(std::round(camera_yaw_offset_deg_ / ALPHA_RES));
  
  for (int i = 0; i < PX4_SECTORS; ++i) {
    // PX4 index i corresponds to angle i * 5° clockwise from front
    // Convert to histogram index:
    // - PX4 angle = i * 5° (0° = front, clockwise positive)
    // - Histogram uses: azimuth = atan2(x, y), where 0° = North (+Y), 90° = East (+X)
    // - But camera data: need camera_offset correction
    
    // PX4 body frame angle (clockwise from front)
    float px4_angle_deg = i * PX4_SECTOR_DEG;
    
    // Convert PX4 angle to histogram index
    // Histogram: index 0 = -180°, index 30 = 0° (for 60 bins, 6° each)
    // PX4 0° (front) should map to histogram index that represents front after camera correction
    // 
    // histogram_angle = px4_angle - 180° (to shift 0° front to histogram's 0° reference)
    // Then add camera offset
    float hist_angle_deg = px4_angle_deg + camera_yaw_offset_deg_ - 180.0f;
    
    // Normalize to [-180, 180)
    while (hist_angle_deg >= 180.0f) hist_angle_deg -= 360.0f;
    while (hist_angle_deg < -180.0f) hist_angle_deg += 360.0f;
    
    // Convert angle to histogram index
    int hist_index = static_cast<int>(std::round((hist_angle_deg + 180.0f) / ALPHA_RES));
    hist_index = hist_index % GRID_LENGTH_Z;
    if (hist_index < 0) hist_index += GRID_LENGTH_Z;
    
    float dist = hist.get_dist(0, hist_index);

    // For 360° sensors (like 2D/3D lidar) or when FOV is not set, skip FOV check
    // FOV check is only meaningful for limited-FOV sensors like depth cameras
    // Mid-360 and similar lidars have h_fov close to 360°, but updateFOVFromMaxima 
    // might not detect it correctly, so we use a lower threshold (180°)
    bool is_360_sensor = fov_fcu_frame_.empty() || 
                         (fov_fcu_frame_.size() == 1 && fov_fcu_frame_[0].h_fov_deg >= 180.0f);
    
    if (is_360_sensor || histogramIndexYawInsideFOV(fov_fcu_frame_, hist_index, position_, yaw_fcu_frame_deg_)) {
      msg.ranges.push_back(dist > min_sensor_range_ ? dist : max_sensor_range_ + 0.01f);
    } else {
      msg.ranges.push_back(NAN);
    }
  }

  distance_data_ = msg;
}

void LocalPlanner::updateObstacleDistanceMsg() {
  sensor_msgs::msg::LaserScan msg = {};
  {
    const auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const int64_t now_ns = now.nanoseconds();
    msg.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
  }
  msg.header.frame_id = "base_link";  // PX4 expects body frame
  msg.angle_increment = static_cast<float>(ALPHA_RES) * M_PI / 180.0f;
  msg.angle_min = -M_PI;  // -180 degrees
  msg.angle_max = M_PI - msg.angle_increment;  // +180 degrees (exclusive)
  msg.time_increment = 0.0f;
  msg.scan_time = 0.1f;  // 10 Hz
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
  px4_.param_acc_up_max = 10.f;
  px4_.param_mpc_z_vel_max_up = 3.f;
  px4_.param_mpc_acc_down_max = 10.f;
  px4_.param_mpc_acc_hor = 5.f;
  px4_.param_mpc_xy_cruise = 3.f;
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
