from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("local_planner")

    default_params = os.path.join(pkg_share, "params", "local_planner.yaml")

    params_file = LaunchConfiguration("params_file")

    # =========================================================================
    # 静态 TF 变换节点：为 Gazebo SITL 的相机帧（x500_depth 模型）提供
    # base_link → 各相机帧 的坐标变换。
    #
    # 相机帧 ID 来自 ros_gz_bridge 桥接后的 PointCloud2.header.frame_id（
    # Gazebo 使用 <model_name>/<link_name>/<sensor_name> 的全限定名）。
    #
    # 位姿参数来源：PX4 autopilot-for-pilot4_v1.15.4 的
    #   Tools/simulation/gz/models/x500_depth/model.sdf
    #   Tools/simulation/gz/models/OakD-Lite/model.sdf
    # 坐标系约定：body frame 为 FLU（X=前，Y=左，Z=上），与 MAVROS base_link 一致。
    # =========================================================================

    # 前置深度相机（OakD-Lite / StereoOV7251）
    # CameraJoint 将 OakD-Lite/base_link 放置于 (0.12, 0.03, 0.242) 处
    # StereoOV7251 传感器在 OakD-Lite/base_link 内偏移 (0.01233, -0.03, 0.01878)
    # 合计：(0.12+0.01233, 0.03-0.03, 0.242+0.01878) = (0.13233, 0, 0.26078)
    tf_front_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_front_camera",
        arguments=[
            "--x", "0.13233",
            "--y", "0.0",
            "--z", "0.26078",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "x500_depth_0/OakD-Lite/base_link/StereoOV7251",
        ],
        output="screen",
    )

    # 后置深度相机（depth_camera_rear）
    # SDF: pose relative_to="base_link" → (-0.12, 0, 0.242, 0, 0, π)
    tf_rear_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_rear_camera",
        arguments=[
            "--x", "-0.12",
            "--y", "0.0",
            "--z", "0.242",
            "--yaw", "3.14159",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "x500_depth_0/depth_camera_rear_link/depth_camera_rear",
        ],
        output="screen",
    )

    # 左置深度相机（depth_camera_left）
    # SDF: pose relative_to="base_link" → (0, 0.12, 0.242, 0, 0, π/2)
    tf_left_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_left_camera",
        arguments=[
            "--x", "0.0",
            "--y", "0.12",
            "--z", "0.242",
            "--yaw", "1.5708",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "x500_depth_0/depth_camera_left_link/depth_camera_left",
        ],
        output="screen",
    )

    # 右置深度相机（depth_camera_right）
    # SDF: pose relative_to="base_link" → (0, -0.12, 0.242, 0, 0, -π/2)
    tf_right_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_right_camera",
        arguments=[
            "--x", "0.0",
            "--y", "-0.12",
            "--z", "0.242",
            "--yaw", "-1.5708",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "x500_depth_0/depth_camera_right_link/depth_camera_right",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameters file for local_planner",
            ),
            Node(
                package="local_planner",
                executable="local_planner_node",
                name="local_planner",
                output="screen",
                parameters=[params_file],
            ),
            tf_front_camera,
            tf_rear_camera,
            tf_left_camera,
            tf_right_camera,
        ]
    )
