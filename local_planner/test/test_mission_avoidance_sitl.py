#!/usr/bin/env python3
"""
PX4 Mission Mode Avoidance SITL Test

This script tests obstacle avoidance in PX4 Mission mode by:
1. Taking off the drone
2. Sending a mission waypoint
3. Publishing simulated obstacle point cloud
4. Monitoring avoidance behavior

Prerequisites:
- PX4 SITL running (make px4_sitl gz_x500_depth)
- MAVROS2 connected
- local_planner running

Usage:
    python3 test_mission_avoidance_sitl.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
import struct
import time
import sys

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, Trajectory
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import PointCloud2, PointField


class MissionAvoidanceSITLTest(Node):
    def __init__(self):
        super().__init__('mission_avoidance_sitl_test')
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.trajectory_received = False
        self.last_trajectory = None
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, sensor_qos)
        self.traj_gen_sub = self.create_subscription(
            Trajectory, '/mavros/trajectory/generated', self.traj_gen_cb, 10)
        
        # Publishers
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/camera/depth/points', 10)
        self.traj_desired_pub = self.create_publisher(
            Trajectory, '/mavros/trajectory/desired', 10)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Wait for services
        self.get_logger().info("Waiting for MAVROS services...")
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("MAVROS services ready")
        
        # Test parameters
        self.target_altitude = 5.0
        self.goal_position = np.array([15.0, 0.0, 5.0])  # Target waypoint
        self.obstacle_distance = 8.0  # Obstacle at 8m in front
        self.obstacle_width = 6.0
        self.obstacle_height = 4.0
        
        # Obstacle publishing timer
        self.obstacle_timer = None
        self.publish_obstacle = False
        
    def state_cb(self, msg):
        self.current_state = msg
        
    def pose_cb(self, msg):
        self.current_pose = msg
        
    def traj_gen_cb(self, msg):
        self.trajectory_received = True
        self.last_trajectory = msg
        if msg.point_valid[0]:
            pos = msg.point_1.position
            self.get_logger().info(
                f"📍 Trajectory generated: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
    
    def get_position(self):
        """Get current position as numpy array"""
        p = self.current_pose.pose.position
        return np.array([p.x, p.y, p.z])
    
    async def arm(self):
        """Arm the drone"""
        self.get_logger().info("Arming...")
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        await future
        if future.result().success:
            self.get_logger().info("✅ Armed")
            return True
        else:
            self.get_logger().error("❌ Arming failed")
            return False
    
    async def set_mode(self, mode):
        """Set flight mode"""
        self.get_logger().info(f"Setting mode to {mode}...")
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        await future
        if future.result().mode_sent:
            self.get_logger().info(f"✅ Mode set to {mode}")
            return True
        else:
            self.get_logger().error(f"❌ Failed to set mode {mode}")
            return False
    
    async def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        self.get_logger().info(f"Taking off to {altitude}m...")
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        await future
        if future.result().success:
            self.get_logger().info("✅ Takeoff command sent")
            return True
        else:
            self.get_logger().error("❌ Takeoff failed")
            return False
    
    def publish_obstacle_cloud(self):
        """Publish simulated obstacle point cloud"""
        if not self.publish_obstacle:
            return
            
        pos = self.get_position()
        stamp = self.get_clock().now().to_msg()
        
        # Create wall obstacle in front of drone (in map frame)
        points = []
        wall_x = pos[0] + self.obstacle_distance
        
        # Generate wall points
        y_samples = np.linspace(-self.obstacle_width/2, self.obstacle_width/2, 30)
        z_samples = np.linspace(
            self.target_altitude - self.obstacle_height/2,
            self.target_altitude + self.obstacle_height/2, 
            20
        )
        
        for y in y_samples:
            for z in z_samples:
                points.append([wall_x, y, z])
        
        # Create PointCloud2 message
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "map"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        
        data = b""
        for p in points:
            data += struct.pack("fff", float(p[0]), float(p[1]), float(p[2]))
        cloud.data = list(data)
        
        self.cloud_pub.publish(cloud)
    
    def publish_mission_trajectory(self):
        """Publish desired trajectory (simulating PX4 mission planner)"""
        stamp = self.get_clock().now().to_msg()
        
        traj = Trajectory()
        traj.header.stamp = stamp
        traj.header.frame_id = "map"
        traj.type = Trajectory.MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS
        
        # Target waypoint
        traj.point_1.position.x = float(self.goal_position[0])
        traj.point_1.position.y = float(self.goal_position[1])
        traj.point_1.position.z = float(self.goal_position[2])
        traj.point_1.velocity.x = 3.0  # Desired velocity
        traj.point_1.velocity.y = 0.0
        traj.point_1.velocity.z = 0.0
        traj.point_1.yaw = 0.0
        traj.point_1.yaw_rate = float('nan')
        
        traj.point_valid[0] = True
        traj.point_valid[1] = False
        traj.point_valid[2] = False
        traj.point_valid[3] = False
        traj.point_valid[4] = False
        
        self.traj_desired_pub.publish(traj)
    
    async def run_test(self):
        """Main test sequence"""
        print("\n" + "=" * 60)
        print("PX4 Mission Mode Avoidance SITL Test")
        print("=" * 60)
        print(f"Target waypoint: {self.goal_position}")
        print(f"Obstacle: {self.obstacle_distance}m ahead, {self.obstacle_width}m wide")
        print("=" * 60 + "\n")
        
        # Wait for connection
        self.get_logger().info("[1/6] Waiting for MAVROS connection...")
        while not self.current_state.connected:
            await self.sleep(0.5)
        self.get_logger().info("✅ MAVROS connected")
        
        # Set to OFFBOARD first to enable control, then ARM
        self.get_logger().info("[2/6] Preparing for takeoff...")
        
        # First set OFFBOARD mode (PX4 requires this before arming in some cases)
        # Actually for SITL, we can use POSCTL or AUTO modes
        
        # ARM
        if not self.current_state.armed:
            success = await self.arm()
            if not success:
                return False
            await self.sleep(1.0)
        
        # Takeoff using AUTO.TAKEOFF or manual climb
        self.get_logger().info("[3/6] Taking off...")
        await self.set_mode("AUTO.TAKEOFF")
        await self.sleep(2.0)
        
        # Wait for altitude
        self.get_logger().info(f"   Climbing to {self.target_altitude}m...")
        timeout = 30.0
        start = time.time()
        while time.time() - start < timeout:
            pos = self.get_position()
            if pos[2] > self.target_altitude - 1.0:
                self.get_logger().info(f"✅ Reached altitude: {pos[2]:.1f}m")
                break
            self.get_logger().info(f"   Current altitude: {pos[2]:.1f}m")
            await self.sleep(2.0)
        
        # Switch to Mission mode
        self.get_logger().info("[4/6] Switching to AUTO.MISSION mode...")
        await self.set_mode("AUTO.MISSION")
        await self.sleep(1.0)
        
        # Start obstacle publishing
        self.get_logger().info("[5/6] Starting obstacle simulation...")
        self.publish_obstacle = True
        self.obstacle_timer = self.create_timer(0.1, self.publish_obstacle_cloud)
        
        # Start publishing mission trajectory
        mission_timer = self.create_timer(0.1, self.publish_mission_trajectory)
        
        # Monitor avoidance behavior
        self.get_logger().info("[6/6] Monitoring avoidance behavior...")
        print("\n" + "-" * 60)
        print("Test running - Press Ctrl+C to stop")
        print("-" * 60 + "\n")
        
        max_y_deviation = 0.0
        test_duration = 30.0
        start = time.time()
        
        while time.time() - start < test_duration:
            pos = self.get_position()
            
            # Check Y deviation (avoidance indicator)
            y_deviation = abs(pos[1])
            if y_deviation > max_y_deviation:
                max_y_deviation = y_deviation
            
            # Log status
            if self.trajectory_received and self.last_trajectory:
                traj_pos = self.last_trajectory.point_1.position
                self.get_logger().info(
                    f"Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
                    f"Trajectory: ({traj_pos.x:.2f}, {traj_pos.y:.2f}, {traj_pos.z:.2f}) | "
                    f"Max Y dev: {max_y_deviation:.2f}m")
            else:
                self.get_logger().info(
                    f"Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
                    f"Waiting for trajectory...")
            
            await self.sleep(1.0)
        
        # Results
        print("\n" + "=" * 60)
        print("TEST RESULTS")
        print("=" * 60)
        print(f"Maximum Y deviation: {max_y_deviation:.2f}m")
        if max_y_deviation > 1.0:
            print("✅ AVOIDANCE DETECTED - Drone deviated to avoid obstacle")
        elif max_y_deviation > 0.3:
            print("⚠️ PARTIAL AVOIDANCE - Some deviation detected")
        else:
            print("❌ NO AVOIDANCE - Drone did not deviate")
        print("=" * 60 + "\n")
        
        # Cleanup
        self.obstacle_timer.cancel()
        mission_timer.cancel()
        
        return max_y_deviation > 0.5
    
    async def sleep(self, duration):
        """Async sleep"""
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)


async def main():
    rclpy.init()
    node = MissionAvoidanceSITLTest()
    
    try:
        success = await node.run_test()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import asyncio
    sys.exit(asyncio.run(main()))
