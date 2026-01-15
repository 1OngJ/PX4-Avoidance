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
        ]
    )
