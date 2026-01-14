#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include "avoidance_ros2/model_parameters.hpp"

namespace avoidance_ros2 {

class Px4ParamBridge : public rclcpp::Node {
 public:
  explicit Px4ParamBridge(const rclcpp::NodeOptions & options);

  ModelParameters get_model_parameters() const;

 private:
  void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);
  void try_initial_pull_and_get();

  // Helpers
  static std::optional<float> as_float(const rcl_interfaces::msg::ParameterValue & v);
  static std::optional<int> as_int(const rcl_interfaces::msg::ParameterValue & v);
  void update_from_name_value(const std::string & name, const rcl_interfaces::msg::ParameterValue & value);

  std::string mavros_param_node_{"/mavros/param"};
  bool do_pull_on_start_{true};
  int pull_timeout_ms_{2000};

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  mutable std::mutex mutex_;
  ModelParameters params_{};
};

}  // namespace avoidance_ros2
