#include "avoidance/avoidance_node.h"

namespace avoidance {

AvoidanceNode::AvoidanceNode(const rclcpp::NodeOptions& options)
    : Node("avoidance_node", options), cmdloop_dt_(0.1), statusloop_dt_(0.2) {
  position_received_ = true;
  should_exit_ = false;

  timeout_termination_ = 15;
  timeout_critical_ = 0.5;
  timeout_startup_ = 5.0;

  mission_item_speed_ = NAN;
  param_cb_mutex_.reset(new std::mutex);
}

AvoidanceNode::~AvoidanceNode() {
  should_exit_ = true;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AvoidanceNode::init() {
  mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>(
      "mavros/companion_process/status", 1);
  px4_param_sub_ = this->create_subscription<mavros_msgs::msg::ParamValue>(
      "mavros/param/param_value", 1,
      std::bind(&AvoidanceNode::px4ParamsCallback, this, std::placeholders::_1));
  mission_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
      "mavros/mission/waypoints", 1,
      std::bind(&AvoidanceNode::missionCallback, this, std::placeholders::_1));
  get_px4_param_client_ = this->create_client<mavros_msgs::srv::ParamGet>("mavros/param/get");

  cmdloop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(cmdloop_dt_),
      std::bind(&AvoidanceNode::cmdLoopCallback, this));

  statusloop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(statusloop_dt_),
      std::bind(&AvoidanceNode::statusLoopCallback, this));

  setSystemStatus(MAV_STATE::MAV_STATE_BOOT);

  worker_ = std::thread(&AvoidanceNode::checkPx4Parameters, this);
}

void AvoidanceNode::cmdLoopCallback() {}

void AvoidanceNode::statusLoopCallback() { publishSystemStatus(); }

void AvoidanceNode::setSystemStatus(MAV_STATE state) { companion_state_ = state; }

MAV_STATE AvoidanceNode::getSystemStatus() { return companion_state_; }

// Publish companion process status
void AvoidanceNode::publishSystemStatus() {
  mavros_msgs::msg::CompanionProcessStatus status_msg;
  status_msg.header.stamp = this->get_clock()->now();
  status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  status_msg.state = static_cast<int>(companion_state_);

  mavros_system_status_pub_->publish(status_msg);
}

void AvoidanceNode::checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover) {
  rclcpp::Duration timeout_termination(std::chrono::nanoseconds(static_cast<int64_t>(timeout_termination_ * 1e9)));
  rclcpp::Duration timeout_critical(std::chrono::nanoseconds(static_cast<int64_t>(timeout_critical_ * 1e9)));
  rclcpp::Duration timeout_startup(std::chrono::nanoseconds(static_cast<int64_t>(timeout_startup_ * 1e9)));

  if (since_last_cloud > timeout_termination && since_start > timeout_termination) {
    setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
    RCLCPP_WARN(this->get_logger(), "\033[1;33m Planner abort: missing required data \n \033[0m");
  } else {
    if (since_last_cloud > timeout_critical && since_start > timeout_startup) {
      if (position_received_) {
        hover = true;
        setSystemStatus(MAV_STATE::MAV_STATE_CRITICAL);
        std::string not_received = "";
      } else {
        RCLCPP_WARN(this->get_logger(), 
            "\033[1;33m Pointcloud timeout: No position received, no WP to output.... \n \033[0m");
      }
    } else {
      if (!hover) setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);
    }
  }
}

void AvoidanceNode::px4ParamsCallback(const mavros_msgs::msg::ParamValue::SharedPtr msg) {
  // collect all px4_ parameters needed for model based trajectory planning
  // when adding new parameter to the struct ModelParameters,
  // add new else if case with correct value type
  auto parse_param_f = [&msg](const std::string& name, float& val, const std::string& param_id) -> bool {
    if (param_id == name) {
      RCLCPP_INFO(rclcpp::get_logger("avoidance_node"), 
          "parameter %s is set from  %f to %f", name.c_str(), val, static_cast<float>(msg->real));
      val = static_cast<float>(msg->real);
      return true;
    }
    return false;
  };

  auto parse_param_i = [&msg](const std::string& name, int& val, const std::string& param_id) -> bool {
    if (param_id == name) {
      RCLCPP_INFO(rclcpp::get_logger("avoidance_node"), 
          "parameter %s is set from %i to %li", name.c_str(), val, msg->integer);
      val = static_cast<int>(msg->integer);
      return true;
    }
    return false;
  };

  // Note: In ROS2, we need to get param_id from somewhere - this may require adjustment
  // based on how the actual mavros ROS2 message is structured
  std::string param_id = "";  // This needs to be adjusted based on actual message structure
  
  // clang-format off
  std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
  parse_param_f("MPC_ACC_DOWN_MAX", px4_.param_mpc_acc_down_max, param_id) ||
  parse_param_f("MPC_ACC_HOR", px4_.param_mpc_acc_hor, param_id) ||
  parse_param_f("MPC_ACC_UP_MAX", px4_.param_mpc_acc_up_max, param_id) ||
  parse_param_i("MPC_AUTO_MODE", px4_.param_mpc_auto_mode, param_id) ||
  parse_param_f("MPC_JERK_MIN", px4_.param_mpc_jerk_min, param_id) ||
  parse_param_f("MPC_JERK_MAX", px4_.param_mpc_jerk_max, param_id) ||
  parse_param_f("MPC_LAND_SPEED", px4_.param_mpc_land_speed, param_id) ||
  parse_param_f("MPC_TKO_SPEED", px4_.param_mpc_tko_speed, param_id) ||
  parse_param_f("MPC_XY_CRUISE", px4_.param_mpc_xy_cruise, param_id) ||
  parse_param_f("MPC_Z_VEL_MAX_DN", px4_.param_mpc_z_vel_max_dn, param_id) ||
  parse_param_f("MPC_Z_VEL_MAX_UP", px4_.param_mpc_z_vel_max_up, param_id) ||
  parse_param_f("CP_DIST", px4_.param_cp_dist, param_id) ||
  parse_param_f("NAV_ACC_RAD", px4_.param_nav_acc_rad, param_id) ||
  parse_param_f("MPC_YAWRAUTO_MAX", px4_.param_mpc_yawrauto_max, param_id);
  // clang-format on
}

void AvoidanceNode::checkPx4Parameters() {
  auto request_param = [this](const std::string& name, float& val) {
    if (!get_px4_param_client_->wait_for_service(std::chrono::seconds(1))) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::ParamGet::Request>();
    request->param_id = name;
    auto future = get_px4_param_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        val = static_cast<float>(response->value.real);
      }
    }
  };
  
  while (!should_exit_ && rclcpp::ok()) {
    bool is_param_not_initialized = true;
    {
      std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
      request_param("MPC_ACC_HOR", px4_.param_mpc_acc_hor);
      request_param("MPC_ACC_DOWN_MAX", px4_.param_mpc_acc_down_max);
      request_param("MPC_ACC_UP_MAX", px4_.param_mpc_acc_up_max);
      request_param("MPC_XY_CRUISE", px4_.param_mpc_xy_cruise);
      request_param("MPC_Z_VEL_MAX_DN", px4_.param_mpc_z_vel_max_dn);
      request_param("MPC_Z_VEL_MAX_UP", px4_.param_mpc_z_vel_max_up);
      request_param("CP_DIST", px4_.param_cp_dist);
      request_param("MPC_LAND_SPEED", px4_.param_mpc_land_speed);
      request_param("MPC_JERK_MAX", px4_.param_mpc_jerk_max);
      request_param("NAV_ACC_RAD", px4_.param_nav_acc_rad);
      request_param("MPC_YAWRAUTO_MAX", px4_.param_mpc_yawrauto_max);

      is_param_not_initialized =
          !std::isfinite(px4_.param_mpc_xy_cruise) || !std::isfinite(px4_.param_cp_dist) ||
          !std::isfinite(px4_.param_mpc_land_speed) || !std::isfinite(px4_.param_nav_acc_rad) ||
          !std::isfinite(px4_.param_mpc_acc_hor) || !std::isfinite(px4_.param_mpc_jerk_max) ||
          !std::isfinite(px4_.param_mpc_acc_down_max) || !std::isfinite(px4_.param_mpc_acc_up_max) ||
          !std::isfinite(px4_.param_mpc_z_vel_max_dn) || !std::isfinite(px4_.param_mpc_z_vel_max_up) ||
          !std::isfinite(px4_.param_mpc_yawrauto_max);
    }

    if (is_param_not_initialized) {
      std::this_thread::sleep_for(std::chrono::seconds(5));
    } else {
      std::this_thread::sleep_for(std::chrono::seconds(30));
    }
  }
}

void AvoidanceNode::missionCallback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
  for (size_t index = 0; index < msg->waypoints.size(); index++) {
    if (msg->waypoints[index].is_current) {
      for (int i = static_cast<int>(index); i >= 0; i--) {
        if (msg->waypoints[i].command == static_cast<int>(MavCommand::MAV_CMD_DO_CHANGE_SPEED) &&
            (msg->waypoints[i].param1 - 1.0f) < FLT_MIN && msg->waypoints[i].param2 > 0.0f) {
          // MAV_CMD_DO_CHANGE_SPEED, speed type: ground speed, speed valid
          mission_item_speed_ = msg->waypoints[i].param2;
          break;
        }
      }
      break;
    }
  }
}

ModelParameters AvoidanceNode::getPX4Parameters() const {
  std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
  ModelParameters px4 = px4_;
  return px4;
}
}  // namespace avoidance
