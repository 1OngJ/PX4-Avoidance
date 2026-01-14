from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('local_planner_ros2'), 'config', 'local_planner.yaml'])

    container = ComposableNodeContainer(
        name='local_planner_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='local_planner_ros2',
                plugin='local_planner_ros2::LocalPlannerComponent',
                name='local_planner',
                parameters=[params],
            ),
            # Optional: run param bridge in the same container.
            ComposableNode(
                package='avoidance_ros2',
                plugin='avoidance_ros2::Px4ParamBridge',
                name='px4_param_bridge',
                parameters=[{
                    'mavros_param_node': '/mavros/param',
                    'do_pull_on_start': True,
                    'pull_timeout_ms': 2000,
                }],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
