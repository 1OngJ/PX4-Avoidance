from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("avoidance")

    # Launch arguments
    world_path_arg = DeclareLaunchArgument(
        "world_path",
        default_value=os.path.join(pkg_share, "sim", "worlds", "outdoor_village_3.world"),
        description="Path to the Gazebo world file",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Whether to show Gazebo GUI",
    )

    ns_arg = DeclareLaunchArgument(
        "ns",
        default_value="/",
        description="Namespace",
    )

    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://:14540@localhost:14557",
        description="FCU URL",
    )

    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="",
        description="GCS URL",
    )

    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system",
        default_value="1",
        description="Target system ID",
    )

    tgt_component_arg = DeclareLaunchArgument(
        "tgt_component",
        default_value="1",
        description="Target component ID",
    )

    # Configuration
    world_path = LaunchConfiguration("world_path")
    gui = LaunchConfiguration("gui")
    ns = LaunchConfiguration("ns")
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")

    # MAVROS node
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=ns,
        output="screen",
        parameters=[
            os.path.join(pkg_share, "resource", "px4_config.yaml"),
            {
                "fcu_url": fcu_url,
                "gcs_url": gcs_url,
                "target_system_id": tgt_system,
                "target_component_id": tgt_component,
            },
        ],
    )

    return LaunchDescription(
        [
            world_path_arg,
            gui_arg,
            ns_arg,
            fcu_url_arg,
            gcs_url_arg,
            tgt_system_arg,
            tgt_component_arg,
            mavros_node,
        ]
    )
