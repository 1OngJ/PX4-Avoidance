#!/bin/bash
# Launch script for local_planner with PX4 SITL simulation
#
# Supported models: x500_lidar_2d, x500_depth
#
# Prerequisites:
#   1. Start PX4 SITL: make px4_sitl gz_x500_depth  (or gz_x500_lidar_2d)
#   2. Start MAVROS2: ros2 launch mavros mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14557
#
# Usage: 
#   ./launch_local_planner_sitl.sh         # Auto-detect model
#   ./launch_local_planner_sitl.sh depth   # Force depth camera mode
#   ./launch_local_planner_sitl.sh lidar   # Force lidar mode

# Ensure script is run with bash (not sh)
if [ -z "$BASH_VERSION" ]; then
    echo "ERROR: This script must be run with bash, not sh"
    echo "Usage: bash $0 $*  OR  ./$0 $*"
    exit 1
fi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

# Detect or select model
MODEL_TYPE="${1:-auto}"

echo "=== Local Planner SITL Launch Script ==="
echo "Workspace: $WS_DIR"

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

# Auto-detect model from Gazebo topics
if [ "$MODEL_TYPE" = "auto" ]; then
    echo "[0/4] Auto-detecting PX4 model..."
    if gz topic -l 2>/dev/null | grep -q "x500_depth"; then
        MODEL_TYPE="depth"
        echo "   Detected: x500_depth (depth camera)"
    elif gz topic -l 2>/dev/null | grep -q "x500_lidar"; then
        MODEL_TYPE="lidar"
        echo "   Detected: x500_lidar_2d (2D lidar)"
    else
        echo "ERROR: Could not detect PX4 model. Make sure Gazebo is running."
        echo "       Use: make px4_sitl gz_x500_depth  OR  make px4_sitl gz_x500_lidar_2d"
        exit 1
    fi
fi

# Select parameters based on model
case "$MODEL_TYPE" in
    depth)
        PARAMS_FILE="$SCRIPT_DIR/params/local_planner_depth.yaml"
        VERIFY_TOPIC="/camera/depth/points"
        echo "   Mode: Depth Camera"
        ;;
    lidar)
        PARAMS_FILE="$SCRIPT_DIR/params/local_planner.yaml"
        VERIFY_TOPIC="/lidar/points"
        echo "   Mode: 2D Lidar"
        ;;
    *)
        echo "ERROR: Unknown model type '$MODEL_TYPE'. Use 'depth' or 'lidar'."
        exit 1
        ;;
esac

# Kill any existing processes
echo "[1/4] Cleaning up existing processes..."
pkill -f "ros_gz_bridge" 2>/dev/null || true
pkill -f "local_planner_node" 2>/dev/null || true
sleep 1

# Start Gazebo-ROS2 bridge for lidar
echo "[2/4] Starting Gazebo-ROS2 bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:="$SCRIPT_DIR/config/gz_bridge.yaml" \
    --log-level warn &
BRIDGE_PID=$!
sleep 2

# Verify bridge is working
if ! ros2 topic list 2>/dev/null | grep -q "$VERIFY_TOPIC"; then
    echo "ERROR: Bridge failed to start! Topic $VERIFY_TOPIC not found."
    kill $BRIDGE_PID 2>/dev/null
    exit 1
fi
echo "   Bridge OK: $VERIFY_TOPIC"

# Start local_planner
echo "[3/4] Starting local_planner node..."
ros2 run local_planner local_planner_node \
    --ros-args --params-file "$PARAMS_FILE" \
    --log-level warn &
LP_PID=$!
sleep 2

# Verify local_planner
if ! ros2 node list 2>/dev/null | grep -q "local_planner"; then
    echo "ERROR: local_planner failed to start!"
    kill $BRIDGE_PID $LP_PID 2>/dev/null
    exit 1
fi
echo "   local_planner OK"

# Configure MAVROS for PX4
echo "[4/5] Configuring MAVROS parameters..."

# Enable TF broadcasting from MAVROS (critical for obstacle detection)
if ros2 node list 2>/dev/null | grep -q "/mavros"; then
    ros2 param set /mavros/local_position tf.send true 2>/dev/null && \
        echo "   Set tf.send=true (enables TF broadcast)" || \
        echo "   WARNING: Could not set tf.send parameter"
else
    echo "   WARNING: MAVROS node not found. Make sure MAVROS is running."
fi

# Configure MAVROS obstacle plugin
if ros2 node list 2>/dev/null | grep -q "/mavros/obstacle"; then
    ros2 param set /mavros/obstacle/send mav_frame "MAV_FRAME_BODY_FRD" 2>/dev/null && \
        echo "   Set mav_frame=MAV_FRAME_BODY_FRD" || \
        echo "   WARNING: Could not set mav_frame parameter"
else
    echo "   WARNING: MAVROS obstacle node not found. Make sure MAVROS is running."
fi

ros2 run tf2_ros static_transform_publisher --x 0.12 --y 0.03 --z 0.242 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id x500_depth_0/OakD-Lite/base_link/StereoOV7251 &

echo "[5/5] System ready!"
echo ""
echo "=== Configuration ==="
echo "Model: $MODEL_TYPE"
echo "Params: $PARAMS_FILE"
echo ""
echo "=== Active Topics ==="
echo "Input:"
if [ "$MODEL_TYPE" = "depth" ]; then
    echo "  - /camera/depth/points (PointCloud2 from depth camera)"
    echo "  - /camera/depth/image_raw (Depth image)"
    echo "  - /camera/color/image_raw (RGB image)"
else
    echo "  - /lidar/points (PointCloud2 from 2D lidar)"
    echo "  - /lidar/scan (LaserScan)"
fi
echo "  - /mavros/local_position/pose"
echo "  - /mavros/trajectory/desired"
echo "Output:"
echo "  - /mavros/setpoint_position/local"
echo "  - /mavros/obstacle/send (OBSTACLE_DISTANCE to PX4)"
echo ""
echo "Press Ctrl+C to stop all processes..."

# Handle Ctrl+C
# cleanup() {
#     echo ""
#     echo "Shutting down..."
#     kill -9 $BRIDGE_PID $LP_PID 2>/dev/null
#     wait $BRIDGE_PID $LP_PID 2>/dev/null
#     echo "Done."
#     exit 0
# }
# trap cleanup SIGINT SIGTERM

# Wait for processes (use wait -n for faster response)
while kill -0 $BRIDGE_PID 2>/dev/null && kill -0 $LP_PID 2>/dev/null; do
    sleep 1
done
echo "A process exited unexpectedly."
cleanup

#ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557