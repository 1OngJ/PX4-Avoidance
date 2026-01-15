#!/usr/bin/env python3
"""
local_planner 消息收发功能测试
验证节点能正确接收位姿/速度并发布 setpoint

运行方式：
  1. 先启动节点: ros2 launch local_planner local_planner_mavros.launch.py
  2. 运行测试: python3 test_message_flow.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud2, PointField
from mavros_msgs.msg import State
import struct
import time
import sys


class LocalPlannerTester(Node):
    def __init__(self):
        super().__init__("local_planner_tester")

        # 使用 BEST_EFFORT 匹配 MAVROS 风格
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 发布模拟的 MAVROS 数据
        self.pose_pub = self.create_publisher(PoseStamped, "/mavros/local_position/pose", qos)
        self.vel_pub = self.create_publisher(TwistStamped, "/mavros/local_position/velocity_local", qos)
        self.state_pub = self.create_publisher(State, "/mavros/state", qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud_in", 10)

        # 订阅 local_planner 的输出
        self.setpoint_received = False
        self.setpoint_sub = self.create_subscription(
            PoseStamped, "/mavros/setpoint_position/local", self.setpoint_callback, 10
        )

        # 发布定时器
        self.timer = self.create_timer(0.1, self.publish_mock_data)
        self.count = 0

        self.get_logger().info("测试节点已启动，开始发送模拟数据...")

    def publish_mock_data(self):
        now = self.get_clock().now().to_msg()

        # 发布位姿
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        # 发布速度
        vel = TwistStamped()
        vel.header.stamp = now
        vel.header.frame_id = "fcu"
        vel.twist.linear.x = 0.0
        vel.twist.linear.y = 0.0
        vel.twist.linear.z = 0.0
        self.vel_pub.publish(vel)

        # 发布状态
        state = State()
        state.header.stamp = now
        state.connected = True
        state.armed = True
        state.mode = "OFFBOARD"
        self.state_pub.publish(state)

        # 发布简单点云（一个障碍物点）
        cloud = self.create_simple_cloud(now)
        self.cloud_pub.publish(cloud)

        self.count += 1
        if self.count % 20 == 0:
            status = "✅ 已收到" if self.setpoint_received else "⏳ 等待中"
            self.get_logger().info(f"已发送 {self.count} 帧数据, setpoint {status}")

    def create_simple_cloud(self, stamp):
        """创建一个简单的点云消息"""
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "fcu"
        cloud.height = 1
        cloud.width = 1
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12
        cloud.is_dense = True
        # 一个在前方5米的障碍物点
        cloud.data = struct.pack("fff", 5.0, 0.0, 2.0)
        return cloud

    def setpoint_callback(self, msg):
        if not self.setpoint_received:
            self.get_logger().info(
                f"✅ 收到 setpoint: x={msg.pose.position.x:.2f}, "
                f"y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
            )
            self.setpoint_received = True


def main():
    rclpy.init()
    node = LocalPlannerTester()

    print("=" * 50)
    print("local_planner 消息流测试")
    print("=" * 50)
    print("发送模拟的 MAVROS 数据，等待 local_planner 响应...")
    print("测试将运行 10 秒...")
    print()

    start = time.time()
    try:
        while rclpy.ok() and (time.time() - start) < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print()
    print("=" * 50)
    if node.setpoint_received:
        print("✅ 测试通过: local_planner 正确处理输入并产生输出")
        result = 0
    else:
        print("❌ 测试失败: 未收到 setpoint 输出")
        print("   可能原因:")
        print("   - local_planner 节点未运行")
        print("   - 话题名称不匹配")
        print("   - planner 内部需要更多数据才能产生输出")
        result = 1
    print("=" * 50)

    node.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
