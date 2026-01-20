#!/usr/bin/env python3
"""
Launch file for local_planner with RViz2 visualization.

Usage:
  ros2 launch local_planner local_planner_rviz.launch.py

This launch file starts:
  - local_planner_node with visualization enabled
  - RViz2 with pre-configured display for local planner visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    local_planner_dir = get_package_share_directory('local_planner')
    
    # Paths
    default_params_file = os.path.join(local_planner_dir, 'params', 'local_planner_depth.yaml')
    default_rviz_config = os.path.join(local_planner_dir, 'resource', 'local_planner.rviz')

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the local planner parameters file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz2 configuration file'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz2 visualization'
    )

    # Local planner node
    local_planner_node = Node(
        package='local_planner',
        executable='local_planner_node',
        name='local_planner',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    return LaunchDescription([
        params_file_arg,
        rviz_config_arg,
        enable_rviz_arg,
        local_planner_node,
        rviz_node,
    ])
