"""Local/headless-safe Host RViz launch using Fast DDS loopback."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Include the remote-observe launch with local Fast DDS defaults."""
    package_share = get_package_share_directory("myarm_m750_visualization")
    remote_launch = os.path.join(
        package_share, "launch", "remote_observe.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "enable_network_probe", default_value="true"
            ),
            DeclareLaunchArgument("report_json_file", default_value=""),
            DeclareLaunchArgument("report_csv_file", default_value=""),
            DeclareLaunchArgument("report_interval_s", default_value="5.0"),
            DeclareLaunchArgument(
                "clock_offset_source",
                default_value="local_loopback_same_clock",
            ),
            DeclareLaunchArgument(
                "measured_clock_offset_ms",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "require_clock_offset_measurement",
                default_value="true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(remote_launch),
                launch_arguments={
                    "network_config_file": os.path.join(
                        package_share, "config", "network_host_local.yaml"
                    ),
                    "fastdds_profile_file": os.path.join(
                        package_share, "config", "fastdds_local.xml"
                    ),
                    "headless": LaunchConfiguration("headless"),
                    "enable_network_probe": LaunchConfiguration(
                        "enable_network_probe"
                    ),
                    "report_json_file": LaunchConfiguration(
                        "report_json_file"
                    ),
                    "report_csv_file": LaunchConfiguration(
                        "report_csv_file"
                    ),
                    "report_interval_s": LaunchConfiguration(
                        "report_interval_s"
                    ),
                    "check_local_interface": "false",
                    "clock_offset_source": LaunchConfiguration(
                        "clock_offset_source"
                    ),
                    "measured_clock_offset_ms": LaunchConfiguration(
                        "measured_clock_offset_ms"
                    ),
                    "require_clock_offset_measurement": LaunchConfiguration(
                        "require_clock_offset_measurement"
                    ),
                }.items(),
            ),
        ]
    )
