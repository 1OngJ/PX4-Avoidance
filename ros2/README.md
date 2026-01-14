# PX4-Avoidance ROS2 (Jazzy) scaffold

This folder contains ROS2 (ament/colcon) **scaffolding** packages for porting the ROS1 PX4 Avoidance stack.

## Packages
- `avoidance_ros2`: minimal utilities + PX4/MAVROS2 parameter bridge skeleton.
- `local_planner_ros2`: ROS2 component node skeleton that wires MAVROS2 topics and parameters.

## Build
```bash
cd <your_colcon_ws>/src
# symlink or copy this repo, or just add the `ros2/` folder to your workspace
colcon build --symlink-install
```

## Run (example)
```bash
source install/setup.bash
# Start a component container
ros2 run rclcpp_components component_container_mt

# In another terminal: load the component
ros2 component load /ComponentManager local_planner_ros2 local_planner_ros2::LocalPlannerComponent
```

You will likely want to use the provided launch file under `local_planner_ros2/launch/` instead.
