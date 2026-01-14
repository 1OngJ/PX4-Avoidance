#include "local_planner_ros2/local_planner_component.hpp"

#include <chrono>

#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace local_planner_ros2 {

using namespace std::chrono_literals;

LocalPlannerComponent::LocalPlannerComponent(const rclcpp::NodeOptions & options)
    : rclcpp::Node("local_planner", options), tf_buffer_(this->get_clock()) {
  declare_and_get_params();

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

  // QoS: match typical MAVROS2 + sensor data reliability.
  auto sensor_qos = rclcpp::SensorDataQoS();

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", sensor_qos, std::bind(&LocalPlannerComponent::on_pose, this, std::placeholders::_1));
  vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", sensor_qos,
      std::bind(&LocalPlannerComponent::on_velocity, this, std::placeholders::_1));
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10), std::bind(&LocalPlannerComponent::on_state, this, std::placeholders::_1));
  traj_sub_ = this->create_subscription<mavros_msgs::msg::Trajectory>(
      "/mavros/trajectory/desired", rclcpp::QoS(10),
      std::bind(&LocalPlannerComponent::on_traj_desired, this, std::placeholders::_1));
  altitude_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>(
      "/mavros/altitude", rclcpp::QoS(10), std::bind(&LocalPlannerComponent::on_altitude, this, std::placeholders::_1));

  goal_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/input/goal_position", rclcpp::QoS(10), std::bind(&LocalPlannerComponent::on_goal_marker, this, std::placeholders::_1));

  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/mavros/obstacle/send", rclcpp::QoS(10));
  setpoint_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", rclcpp::QoS(10));
  traj_generated_pub_ = this->create_publisher<mavros_msgs::msg::Trajectory>("/mavros/trajectory/generated", rclcpp::QoS(10));

  cloud_subs_.resize(pointcloud_topics_.size());
  for (size_t i = 0; i < pointcloud_topics_.size(); ++i) {
    const auto & topic = pointcloud_topics_[i];
    cloud_subs_[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, sensor_qos,
        [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->on_pointcloud(msg, i); });
  }

  cmdloop_timer_ = this->create_wall_timer(100ms, std::bind(&LocalPlannerComponent::cmdloop, this));

  RCLCPP_INFO(this->get_logger(), "local_planner_ros2 scaffold started (pointcloud_topics=%zu)", pointcloud_topics_.size());
}

LocalPlannerComponent::~LocalPlannerComponent() = default;

void LocalPlannerComponent::declare_and_get_params() {
  this->declare_parameter<std::vector<std::string>>("pointcloud_topics", std::vector<std::string>{"/camera/depth/points"});
  this->declare_parameter<std::string>("frame_fcu", frame_fcu_);
  this->declare_parameter<std::string>("frame_local", frame_local_);
  this->declare_parameter<bool>("accept_goal_input_topic", accept_goal_input_topic_);

  pointcloud_topics_ = this->get_parameter("pointcloud_topics").as_string_array();
  frame_fcu_ = this->get_parameter("frame_fcu").as_string();
  frame_local_ = this->get_parameter("frame_local").as_string();
  accept_goal_input_topic_ = this->get_parameter("accept_goal_input_topic").as_bool();
}

void LocalPlannerComponent::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_pose_ = *msg;
  pose_received_ = true;
}

void LocalPlannerComponent::on_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr) {}
void LocalPlannerComponent::on_state(const mavros_msgs::msg::State::SharedPtr) {}
void LocalPlannerComponent::on_traj_desired(const mavros_msgs::msg::Trajectory::SharedPtr) {}
void LocalPlannerComponent::on_altitude(const mavros_msgs::msg::Altitude::SharedPtr) {}

void LocalPlannerComponent::on_goal_marker(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
  if (!accept_goal_input_topic_) return;
  if (msg->markers.empty()) return;
  // Scaffold: keep hook for goal updates.
}

void LocalPlannerComponent::on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr, size_t) {
  // Scaffold: hook pointcloud ingestion here.
  // In the full port, this is where you'll:
  // - transform cloud into frame_local_ using tf2
  // - feed planner
  // - publish LaserScan + setpoint/trajectory
}

void LocalPlannerComponent::cmdloop() {
  // Scaffold behavior:
  // - If pose exists, publish a dummy setpoint echoing current pose (so you can verify the wiring).
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!pose_received_) return;
    pose = last_pose_;
  }

  pose.header.stamp = this->get_clock()->now();
  setpoint_pos_pub_->publish(pose);

  // Publish an "empty" LaserScan periodically (lets you verify /mavros/obstacle/send path).
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = this->get_clock()->now();
  scan.header.frame_id = frame_fcu_;
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;
  obstacle_pub_->publish(scan);
}

}  // namespace local_planner_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(local_planner_ros2::LocalPlannerComponent)
