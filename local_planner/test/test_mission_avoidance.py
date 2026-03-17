#!/usr/bin/env python3
"""
Mission 模式避障功能测试

测试场景：
  1. 模拟无人机在 (0, 0, 3) 位置
  2. PX4 发送目标航点 (10, 0, 3) - 直线前进
  3. 在前方 5m 处放置一面障碍物墙
  4. 验证 local_planner 是否生成绕行轨迹

运行方式：
  1. 启动 local_planner:
     ros2 launch local_planner local_planner_mavros.launch.py \
         params_file:=/path/to/local_planner_depth.yaml
  2. 运行测试:
     python3 test_mission_avoidance.py

预期结果：
  - /mavros/trajectory/generated 应该发布修改后的轨迹
  - 当遇到障碍物时，生成的航点应该偏离直线路径
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from mavros_msgs.msg import State, Trajectory, PositionTarget
from std_msgs.msg import Header
import struct
import time
import sys
import math
import numpy as np


class MissionAvoidanceTester(Node):
    """Mission 模式避障测试节点"""

    def __init__(self):
        super().__init__("mission_avoidance_tester")

        # ============ QoS 配置 ============
        # MAVROS2 使用 BEST_EFFORT 发布 pose/velocity
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # State 使用 TRANSIENT_LOCAL (latched)
        state_qos = QoSProfile(
            depth=10, 
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        # Trajectory 使用 RELIABLE
        traj_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ============ 发布者 - 模拟 MAVROS/PX4 数据 ============
        self.pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", sensor_qos)
        self.vel_pub = self.create_publisher(
            TwistStamped, "/mavros/local_position/velocity_local", sensor_qos)
        self.state_pub = self.create_publisher(
            State, "/mavros/state", state_qos)
        self.traj_desired_pub = self.create_publisher(
            Trajectory, "/mavros/trajectory/desired", traj_qos)
        
        # 点云发布 - 模拟深度相机
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/camera/depth/points", 10)

        # ============ 订阅者 - 接收 local_planner 输出 ============
        self.traj_generated_sub = self.create_subscription(
            Trajectory, "/mavros/trajectory/generated", 
            self.trajectory_generated_callback, traj_qos)
        self.setpoint_sub = self.create_subscription(
            PoseStamped, "/mavros/setpoint_position/local",
            self.setpoint_callback, 10)
        self.obstacle_sub = self.create_subscription(
            LaserScan, "/mavros/obstacle/send",
            self.obstacle_callback, 10)

        # ============ 测试状态 ============
        self.drone_position = np.array([0.0, 0.0, 3.0])  # 无人机位置
        self.drone_yaw = 0.0  # 朝向 +X 方向
        self.goal_position = np.array([10.0, 0.0, 3.0])  # 目标航点
        
        # 障碍物配置：在前方 5m 处放置一面墙
        self.obstacle_distance = 5.0  # 障碍物距离
        self.obstacle_width = 4.0     # 障碍物宽度 (Y方向)
        self.obstacle_height = 2.0    # 障碍物高度 (Z方向)
        
        # 测试结果收集
        self.trajectory_received = False
        self.setpoint_received = False
        self.obstacle_data_received = False
        self.generated_trajectories = []
        self.generated_setpoints = []
        self.obstacle_distances = []
        
        # 计数器
        self.publish_count = 0
        self.test_start_time = time.time()

        # 定时器 - 10Hz 发布模拟数据
        self.timer = self.create_timer(0.1, self.publish_mock_data)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Mission 模式避障测试已启动")
        self.get_logger().info(f"无人机位置: {self.drone_position}")
        self.get_logger().info(f"目标航点: {self.goal_position}")
        self.get_logger().info(f"障碍物距离: {self.obstacle_distance}m")
        self.get_logger().info("=" * 60)

    def publish_mock_data(self):
        """发布模拟的传感器和 PX4 数据"""
        now = self.get_clock().now().to_msg()

        # 1. 发布无人机位姿
        self.publish_pose(now)

        # 2. 发布速度
        self.publish_velocity(now)

        # 3. 发布飞行状态 (AUTO.MISSION 模式)
        self.publish_state(now)

        # 4. 发布期望轨迹 (模拟 PX4 任务规划器)
        self.publish_trajectory_desired(now)

        # 5. 发布障碍物点云
        self.publish_obstacle_cloud(now)

        self.publish_count += 1
        
        # 每 2 秒打印一次状态
        if self.publish_count % 20 == 0:
            self.print_status()

    def publish_pose(self, stamp):
        """发布无人机位姿"""
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = float(self.drone_position[0])
        pose.pose.position.y = float(self.drone_position[1])
        pose.pose.position.z = float(self.drone_position[2])
        # 朝向 +X 方向 (yaw = 0)
        pose.pose.orientation.w = math.cos(self.drone_yaw / 2)
        pose.pose.orientation.z = math.sin(self.drone_yaw / 2)
        self.pose_pub.publish(pose)

    def publish_velocity(self, stamp):
        """发布速度 (悬停状态)"""
        vel = TwistStamped()
        vel.header.stamp = stamp
        vel.header.frame_id = "map"
        vel.twist.linear.x = 0.0
        vel.twist.linear.y = 0.0
        vel.twist.linear.z = 0.0
        self.vel_pub.publish(vel)

    def publish_state(self, stamp):
        """发布飞行状态 - AUTO.MISSION 模式"""
        state = State()
        state.header.stamp = stamp
        state.connected = True
        state.armed = True
        state.guided = True
        state.mode = "AUTO.MISSION"  # Mission 模式，不是 OFFBOARD
        self.state_pub.publish(state)

    def publish_trajectory_desired(self, stamp):
        """发布 PX4 期望轨迹 (模拟任务规划器输出)"""
        traj = Trajectory()
        traj.header.stamp = stamp
        traj.header.frame_id = "map"
        traj.type = Trajectory.MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS

        # point_1: 当前目标航点
        traj.point_1.position.x = float(self.goal_position[0])
        traj.point_1.position.y = float(self.goal_position[1])
        traj.point_1.position.z = float(self.goal_position[2])
        traj.point_1.velocity.x = 2.0  # 期望速度 2 m/s
        traj.point_1.velocity.y = 0.0
        traj.point_1.velocity.z = 0.0
        traj.point_1.yaw = 0.0  # 朝向 +X
        traj.point_1.yaw_rate = float('nan')

        # point_valid 标记
        traj.point_valid[0] = True
        traj.point_valid[1] = False
        traj.point_valid[2] = False
        traj.point_valid[3] = False
        traj.point_valid[4] = False

        self.traj_desired_pub.publish(traj)

    def publish_obstacle_cloud(self, stamp):
        """发布障碍物点云 - 模拟前方的一面墙
        
        重要：local_planner 的 processPointcloud() 函数会计算：
            distanceSq = (position - xyz).squaredNorm()
        其中 position 是 map 坐标系中的无人机位置。
        
        因此点云必须在 map 坐标系中，而不是机体坐标系！
        """
        # 创建一面墙的点云 - 使用 MAP 坐标系
        # 墙的位置：X = drone_x + obstacle_distance (世界坐标)
        # 墙的范围：Y = [-obstacle_width/2, obstacle_width/2]
        #          Z = [drone_z - obstacle_height/2, drone_z + obstacle_height/2]
        
        points = []
        wall_x = self.drone_position[0] + self.obstacle_distance
        
        # 生成墙面点云 (网格采样) - 在 map 坐标系中
        y_samples = np.linspace(-self.obstacle_width/2, self.obstacle_width/2, 20)
        z_samples = np.linspace(
            self.drone_position[2] - self.obstacle_height/2,
            self.drone_position[2] + self.obstacle_height/2, 
            10
        )
        
        for y in y_samples:
            for z in z_samples:
                # 直接使用世界坐标，因为 local_planner 会用 position (map坐标) 来计算距离
                points.append([wall_x, y, z])

        # frame_id 设为 map，点云坐标就是 map 坐标系中的绝对位置
        cloud = self.create_pointcloud2(stamp, points, frame_id="map")
        self.cloud_pub.publish(cloud)

    def create_pointcloud2(self, stamp, points, frame_id="map"):
        """创建 PointCloud2 消息"""
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = frame_id  # 使用指定的坐标系
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
        
        # 打包点云数据
        data = b""
        for p in points:
            data += struct.pack("fff", float(p[0]), float(p[1]), float(p[2]))
        cloud.data = list(data)
        
        return cloud

    def trajectory_generated_callback(self, msg):
        """接收 local_planner 生成的避障轨迹"""
        self.trajectory_received = True
        
        # 记录生成的轨迹点
        if msg.point_valid[0]:
            wp = {
                'x': msg.point_1.position.x,
                'y': msg.point_1.position.y,
                'z': msg.point_1.position.z,
                'vx': msg.point_1.velocity.x,
                'vy': msg.point_1.velocity.y,
                'vz': msg.point_1.velocity.z,
            }
            self.generated_trajectories.append(wp)
            
            # 首次收到时打印
            if len(self.generated_trajectories) == 1:
                self.get_logger().info(
                    f"✅ 收到 trajectory/generated: "
                    f"pos=({wp['x']:.2f}, {wp['y']:.2f}, {wp['z']:.2f})"
                )

    def setpoint_callback(self, msg):
        """接收 setpoint (OFFBOARD 模式用)"""
        self.setpoint_received = True
        wp = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
        }
        self.generated_setpoints.append(wp)

    def obstacle_callback(self, msg):
        """接收障碍物距离数据"""
        self.obstacle_data_received = True
        
        # 记录前方扇区的障碍物距离 (大约 0° 方向)
        if len(msg.ranges) > 0:
            # LaserScan 的 angle_min 到 angle_max 对应不同方向
            # 找到最小距离
            valid_ranges = [r for r in msg.ranges if not math.isnan(r) and r > 0.1]
            if valid_ranges:
                min_dist = min(valid_ranges)
                self.obstacle_distances.append(min_dist)
                
                if len(self.obstacle_distances) == 1:
                    self.get_logger().info(f"✅ 收到障碍物数据，最近距离: {min_dist:.2f}m")

    def print_status(self):
        """打印当前测试状态"""
        elapsed = time.time() - self.test_start_time
        self.get_logger().info(
            f"[{elapsed:.1f}s] 已发送 {self.publish_count} 帧 | "
            f"trajectory: {'✅' if self.trajectory_received else '❌'} | "
            f"setpoint: {'✅' if self.setpoint_received else '❌'} | "
            f"obstacle: {'✅' if self.obstacle_data_received else '❌'}"
        )

    def analyze_results(self):
        """分析测试结果"""
        print("\n" + "=" * 60)
        print("测试结果分析")
        print("=" * 60)
        
        # 1. 检查是否收到输出
        print(f"\n📊 数据接收情况:")
        print(f"   - trajectory/generated: {'✅ 收到' if self.trajectory_received else '❌ 未收到'} "
              f"({len(self.generated_trajectories)} 条)")
        print(f"   - setpoint/local: {'✅ 收到' if self.setpoint_received else '❌ 未收到'} "
              f"({len(self.generated_setpoints)} 条)")
        print(f"   - obstacle/send: {'✅ 收到' if self.obstacle_data_received else '❌ 未收到'} "
              f"({len(self.obstacle_distances)} 条)")
        
        # 2. 分析轨迹是否有避障行为
        print(f"\n🎯 避障行为分析:")
        
        if self.generated_trajectories:
            # 计算生成的轨迹与直线路径的偏差
            direct_path_y = 0.0  # 直线路径 Y = 0
            
            y_values = [t['y'] for t in self.generated_trajectories]
            max_y_deviation = max(abs(y) for y in y_values) if y_values else 0
            avg_y_deviation = sum(abs(y) for y in y_values) / len(y_values) if y_values else 0
            
            print(f"   - 最大 Y 偏移: {max_y_deviation:.3f}m")
            print(f"   - 平均 Y 偏移: {avg_y_deviation:.3f}m")
            
            # 检查是否有明显的绕行
            if max_y_deviation > 0.5:
                print(f"   - ✅ 检测到绕行行为 (偏移 > 0.5m)")
                avoidance_detected = True
            else:
                print(f"   - ⚠️ 未检测到明显绕行 (偏移 < 0.5m)")
                avoidance_detected = False
                
            # 检查 X 方向进展
            x_values = [t['x'] for t in self.generated_trajectories]
            if x_values:
                x_progress = max(x_values) - self.drone_position[0]
                print(f"   - X 方向进展: {x_progress:.2f}m")
        else:
            print("   - ❌ 没有收到任何轨迹数据")
            avoidance_detected = False
        
        # 3. 检查障碍物检测
        print(f"\n🚧 障碍物检测:")
        if self.obstacle_distances:
            min_detected = min(self.obstacle_distances)
            avg_detected = sum(self.obstacle_distances) / len(self.obstacle_distances)
            print(f"   - 检测到的最近距离: {min_detected:.2f}m")
            print(f"   - 平均检测距离: {avg_detected:.2f}m")
            print(f"   - 实际障碍物距离: {self.obstacle_distance:.2f}m")
            
            if abs(min_detected - self.obstacle_distance) < 1.0:
                print(f"   - ✅ 障碍物距离检测准确 (误差 < 1m)")
            else:
                print(f"   - ⚠️ 障碍物距离检测偏差较大")
        else:
            print("   - ❌ 没有收到障碍物距离数据")
        
        # 4. 总结
        print(f"\n📋 测试总结:")
        passed = self.trajectory_received and self.obstacle_data_received
        
        if passed:
            print("   ✅ Mission 模式避障基本功能正常")
            if avoidance_detected:
                print("   ✅ 检测到避障绕行行为")
            else:
                print("   ⚠️ 未检测到明显避障，可能原因:")
                print("      - 障碍物未被正确感知")
                print("      - 规划器未触发避障逻辑")
                print("      - 需要更多时间才能看到绕行")
        else:
            print("   ❌ Mission 模式避障功能异常")
            if not self.trajectory_received:
                print("      - 未收到 trajectory/generated")
                print("      - 检查 local_planner 是否正确订阅 trajectory/desired")
            if not self.obstacle_data_received:
                print("      - 未收到 obstacle/send")
                print("      - 检查点云是否被正确处理")
        
        print("=" * 60)
        
        return passed


def main():
    rclpy.init()
    tester = MissionAvoidanceTester()

    print("\n" + "=" * 60)
    print("PX4 Mission 模式避障功能测试")
    print("=" * 60)
    print("测试配置:")
    print(f"  - 无人机位置: (0, 0, 3)")
    print(f"  - 目标航点: (10, 0, 3)")
    print(f"  - 障碍物: 前方 5m 处的墙")
    print(f"  - 测试时长: 15 秒")
    print()
    print("等待 local_planner 响应...")
    print("=" * 60 + "\n")

    # 运行测试 15 秒
    test_duration = 15.0
    start_time = time.time()
    
    try:
        while rclpy.ok() and (time.time() - start_time) < test_duration:
            rclpy.spin_once(tester, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n测试被用户中断")

    # 分析结果
    passed = tester.analyze_results()

    tester.destroy_node()
    rclpy.shutdown()
    
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
