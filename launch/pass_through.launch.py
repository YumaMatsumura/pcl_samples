import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pass_through_node = ComposableNode(
        name='',
        package='pcl_samples',
        plugin='pcl_filters::PassThrough'
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name='pcl_container',
            namespace='pcl',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                pass_through_node
            ],
            output='screen'
        )
    ])
