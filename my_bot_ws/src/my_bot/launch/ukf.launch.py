import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("my_bot"),
        "config",
        "ukf.yaml",
    )

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
