#pragma once

#include <cmath>

namespace avoidance_ros2 {

// ROS2-friendly copy of the key PX4 parameters used by the planner.
// Keep this header ROS-agnostic so algorithm code can depend on it.
struct ModelParameters {
  int mpc_auto_mode = -1;

  float mpc_jerk_min = NAN;
  float mpc_jerk_max = NAN;
  float mpc_acc_up_max = NAN;
  float mpc_z_vel_max_up = NAN;
  float mpc_acc_down_max = NAN;
  float mpc_z_vel_max_dn = NAN;
  float mpc_acc_hor = NAN;
  float mpc_xy_cruise = NAN;
  float mpc_tko_speed = NAN;
  float mpc_land_speed = NAN;
  float mpc_yawrauto_max = NAN;

  float nav_acc_rad = NAN;

  float cp_dist = NAN;
};

}  // namespace avoidance_ros2
