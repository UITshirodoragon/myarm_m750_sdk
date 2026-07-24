"""Jetson-side robot bring-up: driver plus robot_state_publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("myarm_m750_description")
    urdf_path = os.path.join(
        description_share, "urdf", "myarm_m750_standalone.urdf"
    )
    with open(urdf_path, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    core_config_file = LaunchConfiguration("core_config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "core_config_file",
                description=(
                    "Absolute Jetson path to pycore/config/default.yaml "
                    "or default_real.yaml"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="myarm_m750_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
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
