#include "avoidance_ros2/px4_param_bridge.hpp"

#include <chrono>

#include <rclcpp/parameter_client.hpp>

namespace avoidance_ros2 {

using namespace std::chrono_literals;

Px4ParamBridge::Px4ParamBridge(const rclcpp::NodeOptions & options) : rclcpp::Node("px4_param_bridge", options) {
  this->declare_parameter<std::string>("mavros_param_node", mavros_param_node_);
  this->declare_parameter<bool>("do_pull_on_start", do_pull_on_start_);
  this->declare_parameter<int>("pull_timeout_ms", pull_timeout_ms_);

  mavros_param_node_ = this->get_parameter("mavros_param_node").as_string();
  do_pull_on_start_ = this->get_parameter("do_pull_on_start").as_bool();
  pull_timeout_ms_ = this->get_parameter("pull_timeout_ms").as_int();

  param_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/mavros/param/event", rclcpp::QoS(10),
      std::bind(&Px4ParamBridge::on_parameter_event, this, std::placeholders::_1));

  // Defer initial pull/get until the node is fully up.
  init_timer_ = this->create_wall_timer(500ms, std::bind(&Px4ParamBridge::try_initial_pull_and_get, this));
}

ModelParameters Px4ParamBridge::get_model_parameters() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return params_;
}

std::optional<float> Px4ParamBridge::as_float(const rcl_interfaces::msg::ParameterValue & v) {
  switch (v.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      return static_cast<float>(v.double_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      return static_cast<float>(v.integer_value);
    default:
      return std::nullopt;
  }
}

std::optional<int> Px4ParamBridge::as_int(const rcl_interfaces::msg::ParameterValue & v) {
  switch (v.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      return static_cast<int>(v.integer_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      return static_cast<int>(v.double_value);
    default:
      return std::nullopt;
  }
}

void Px4ParamBridge::update_from_name_value(const std::string & name,
                                           const rcl_interfaces::msg::ParameterValue & value) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Keep names identical to PX4 params (ROS1 code in avoidance_node.cpp did string matches).
  if (name == "MPC_AUTO_MODE") {
    if (auto v = as_int(value)) params_.mpc_auto_mode = *v;
    return;
  }

  auto f = as_float(value);
  if (!f) return;

  if (name == "MPC_JERK_MIN") params_.mpc_jerk_min = *f;
  else if (name == "MPC_JERK_MAX") params_.mpc_jerk_max = *f;
  else if (name == "MPC_ACC_UP_MAX") params_.mpc_acc_up_max = *f;
  else if (name == "MPC_Z_VEL_MAX_UP") params_.mpc_z_vel_max_up = *f;
  else if (name == "MPC_ACC_DOWN_MAX") params_.mpc_acc_down_max = *f;
  else if (name == "MPC_Z_VEL_MAX_DN") params_.mpc_z_vel_max_dn = *f;
  else if (name == "MPC_ACC_HOR") params_.mpc_acc_hor = *f;
  else if (name == "MPC_XY_CRUISE") params_.mpc_xy_cruise = *f;
  else if (name == "MPC_TKO_SPEED") params_.mpc_tko_speed = *f;
  else if (name == "MPC_LAND_SPEED") params_.mpc_land_speed = *f;
  else if (name == "MPC_YAWRAUTO_MAX") params_.mpc_yawrauto_max = *f;
  else if (name == "NAV_ACC_RAD") params_.nav_acc_rad = *f;
  else if (name == "CP_DIST") params_.cp_dist = *f;
}

void Px4ParamBridge::on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
  // Only care about changes coming from mavros param node.
  // Note: msg->node is the fully-qualified node name in ROS2.
  if (!msg->node.empty() && msg->node != mavros_param_node_) {
    // Many systems publish lots of parameter events; avoid doing work for unrelated ones.
    return;
  }

  for (const auto & p : msg->new_parameters) {
    update_from_name_value(p.name, p.value);
  }
  for (const auto & p : msg->changed_parameters) {
    update_from_name_value(p.name, p.value);
  }
}

void Px4ParamBridge::try_initial_pull_and_get() {
  init_timer_->cancel();

  auto client = std::make_shared<rclcpp::SyncParametersClient>(this->shared_from_this(), mavros_param_node_);
  if (!client->wait_for_service(std::chrono::milliseconds(pull_timeout_ms_))) {
    RCLCPP_WARN(this->get_logger(), "MAVROS param services not available on %s; skipping initial get",
                mavros_param_node_.c_str());
    return;
  }

  if (do_pull_on_start_) {
    // MAVROS2 provides /mavros/param/pull.
    auto pull_client = this->create_client<mavros_msgs::srv::ParamPull>("/mavros/param/pull");
    if (pull_client->wait_for_service(std::chrono::milliseconds(pull_timeout_ms_))) {
      auto req = std::make_shared<mavros_msgs::srv::ParamPull::Request>();
      auto future = pull_client->async_send_request(req);
      auto rc = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                                   std::chrono::milliseconds(pull_timeout_ms_));
      if (rc != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Param pull did not complete successfully");
      }
    }
  }

  // Populate initial values (best-effort).
  const std::vector<std::string> names = {
      "MPC_AUTO_MODE", "MPC_JERK_MIN", "MPC_JERK_MAX", "MPC_ACC_UP_MAX", "MPC_Z_VEL_MAX_UP",
      "MPC_ACC_DOWN_MAX", "MPC_Z_VEL_MAX_DN", "MPC_ACC_HOR", "MPC_XY_CRUISE", "MPC_TKO_SPEED",
      "MPC_LAND_SPEED", "MPC_YAWRAUTO_MAX", "NAV_ACC_RAD", "CP_DIST"};

  auto values = client->get_parameters(names);
  for (size_t i = 0; i < names.size() && i < values.size(); ++i) {
    update_from_name_value(names[i], values[i].get_parameter_value());
  }
}

}  // namespace avoidance_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(avoidance_ros2::Px4ParamBridge)
