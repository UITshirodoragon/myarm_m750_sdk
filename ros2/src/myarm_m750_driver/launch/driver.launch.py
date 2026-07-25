"""Launch the lifecycle-equivalent Jetson-side driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create a mock-safe driver launch description."""
    package_share = get_package_share_directory("myarm_m750_driver")
    default_parameters = os.path.join(
        package_share, "config", "robot_mock.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "core_config_file",
                description="Absolute path to a validated Python Core v0.2 profile.",
            ),
            DeclareLaunchArgument("use_real_hardware", default_value="false"),
            DeclareLaunchArgument(
                "enable_command_interfaces", default_value="false"
            ),
            DeclareLaunchArgument("auto_configure", default_value="true"),
            DeclareLaunchArgument("auto_activate", default_value="true"),
            Node(
                package="myarm_m750_driver",
                executable="driver_node",
                name="myarm_m750_driver",
                output="screen",
                parameters=[
                    default_parameters,
                    {
                        "core_config_file": LaunchConfiguration(
                            "core_config_file"
                        ),
                        "use_real_hardware": LaunchConfiguration(
                            "use_real_hardware"
                        ),
                        "enable_command_interfaces": LaunchConfiguration(
                            "enable_command_interfaces"
                        ),
                        "auto_configure": LaunchConfiguration("auto_configure"),
                        "auto_activate": LaunchConfiguration("auto_activate"),
                    },
                ],
            ),
        ]
    )
