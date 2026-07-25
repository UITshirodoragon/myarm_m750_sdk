import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("myarm_m750_camera")
    default_parameters = os.path.join(
        package_share,
        "config",
        "camera_bridge.yaml",
    )
    enable = LaunchConfiguration("enable_camera")
    config_file = LaunchConfiguration("core_camera_config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_camera", default_value="false"),
            DeclareLaunchArgument("core_camera_config_file", default_value=""),
            Node(
                package="myarm_m750_camera",
                executable="camera_bridge",
                name="myarm_m750_camera_bridge",
                condition=IfCondition(enable),
                parameters=[
                    default_parameters,
                    {
                        "core_camera_config_file": config_file,
                    }
                ],
                output="screen",
            ),
        ]
    )
