"""Launch only the Jetson-side MyArm M750 driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    core_config_file = LaunchConfiguration("core_config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "core_config_file",
                description=(
                    "Absolute path to pycore/config/default.yaml or default_real.yaml"
                ),
            ),
            Node(
                package="myarm_m750_driver",
                executable="driver_node",
                name="myarm_m750_driver",
                output="screen",
                parameters=[{"core_config_file": core_config_file}],
            ),
        ]
    )
