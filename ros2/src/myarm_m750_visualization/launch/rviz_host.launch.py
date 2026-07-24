"""Host-PC RViz2 and optional marker node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    rviz_config = os.path.join(
        get_package_share_directory("myarm_m750_visualization"),
        "rviz",
        "robot_host.rviz",
    )
    enable_markers = LaunchConfiguration("enable_markers")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_markers", default_value="true"),
            Node(
                package="myarm_m750_visualization",
                executable="marker_node",
                name="myarm_m750_marker_node",
                output="screen",
                condition=IfCondition(enable_markers),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_myarm_m750_host",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
