"""Local mock-safe Jetson role using Fast DDS loopback."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Include robot.launch.py with a checked-in loopback network profile."""
    bringup_share = get_package_share_directory("myarm_m750_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "core_config_file",
                description=(
                    "Absolute validated mock Python Core v0.2 profile."
                ),
            ),
            DeclareLaunchArgument(
                "enable_command_interfaces", default_value="false"
            ),
            DeclareLaunchArgument("auto_configure", default_value="true"),
            DeclareLaunchArgument("auto_activate", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "robot.launch.py")
                ),
                launch_arguments={
                    "core_config_file": LaunchConfiguration(
                        "core_config_file"
                    ),
                    "network_environment_file": os.path.join(
                        bringup_share,
                        "config",
                        "network_jetson_local.env",
                    ),
                    "use_real_hardware": "false",
                    "enable_command_interfaces": LaunchConfiguration(
                        "enable_command_interfaces"
                    ),
                    "auto_configure": LaunchConfiguration("auto_configure"),
                    "auto_activate": LaunchConfiguration("auto_activate"),
                }.items(),
            ),
        ]
    )
