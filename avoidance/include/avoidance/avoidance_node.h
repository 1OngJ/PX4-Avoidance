#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/param_value.hpp>
#include <mavros_msgs/srv/param_get.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include "avoidance/common.h"
#include "mavros_msgs/msg/companion_process_status.hpp"

#include <thread>
#include <memory>

namespace avoidance {

class AvoidanceNode : public rclcpp::Node {
 public:
  AvoidanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AvoidanceNode();
  
  /**
  * @brief      check healthiness of the avoidance system to trigger failsafe in
  *             the FCU
  * @param[in]  since_last_cloud, time elapsed since the last waypoint was
  *             published to the FCU
  * @param[in]  since_start, time elapsed since staring the node
  * @param[out] planner_is_healthy, true if the planner is running without
  *errors
  * @param[out] hover, true if the vehicle is hovering
  **/
  void checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover);

  ModelParameters getPX4Parameters() const;
  float getMissionItemSpeed() const { return mission_item_speed_; }
  MAV_STATE getSystemStatus();

  /**
  * @brief     polls PX4 Firmware paramters every 30 seconds
  **/
  void checkPx4Parameters();

  void setSystemStatus(MAV_STATE state);
  void init();

  /**
  * @brief     callaback with the list of FCU Mission Items
  * @param[in] msg, list of mission items
  **/
  void missionCallback(const mavros_msgs::msg::WaypointList::SharedPtr msg);

 private:
  rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;

  rclcpp::Subscription<mavros_msgs::msg::ParamValue>::SharedPtr px4_param_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mission_sub_;

  rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedPtr get_px4_param_client_;

  rclcpp::TimerBase::SharedPtr cmdloop_timer_;
  rclcpp::TimerBase::SharedPtr statusloop_timer_;

  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_STANDBY;

  ModelParameters px4_;  // PX4 Firmware paramters
  std::unique_ptr<std::mutex> param_cb_mutex_;

  std::thread worker_;

  double cmdloop_dt_;
  double statusloop_dt_;
  double timeout_termination_;
  double timeout_critical_;
  double timeout_startup_;

  bool position_received_;
  bool should_exit_;

  float mission_item_speed_;

  void cmdLoopCallback();
  void statusLoopCallback();
  void publishSystemStatus();

  /**
  * @brief     callaback with the list of FCU parameters
  * @param[in] msg, list of paramters
  **/
  void px4ParamsCallback(const mavros_msgs::msg::ParamValue::SharedPtr msg);
};
}  // namespace avoidance
#endif  // AVOIDANCE_AVOIDANCE_NODE_H
