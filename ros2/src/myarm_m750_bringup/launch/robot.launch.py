"""Explicit Jetson role: core driver, safety, TF, state, and diagnostics."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ALLOWED_NETWORK_KEYS = {
    "MYARM_M750_ROLE",
    "MYARM_M750_WLAN_INTERFACE",
    "RMW_IMPLEMENTATION",
    "ROS_DOMAIN_ID",
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "ROS_DISCOVERY_SERVER",
}


def _load_network_environment(environment_file):
    source_path = Path(environment_file).expanduser().resolve()
    environment = {}
    for line_number, raw_line in enumerate(
        source_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if not separator or not key or not value:
            raise RuntimeError(
                f"Invalid network environment at line {line_number}."
            )
        if key not in _ALLOWED_NETWORK_KEYS:
            raise RuntimeError(f"Unsupported network environment key: {key}.")
        environment[key] = value
    required = {
        "MYARM_M750_ROLE",
        "MYARM_M750_WLAN_INTERFACE",
        "RMW_IMPLEMENTATION",
        "ROS_DOMAIN_ID",
        "FASTRTPS_DEFAULT_PROFILES_FILE",
    }
    missing = sorted(required - set(environment))
    if missing:
        raise RuntimeError(
            f"Network environment is missing keys: {missing}."
        )
    if environment["MYARM_M750_ROLE"] != "jetson":
        raise RuntimeError("Network environment role must be 'jetson'.")
    if environment["RMW_IMPLEMENTATION"] != "rmw_fastrtps_cpp":
        raise RuntimeError("RMW_IMPLEMENTATION must be rmw_fastrtps_cpp.")
    domain_id = int(environment["ROS_DOMAIN_ID"])
    if domain_id < 0 or domain_id > 232:
        raise RuntimeError("ROS_DOMAIN_ID must be in [0, 232].")
    profile_path = Path(environment["FASTRTPS_DEFAULT_PROFILES_FILE"])
    if not profile_path.is_absolute():
        profile_path = source_path.parent / profile_path
    profile_path = profile_path.resolve()
    if not profile_path.is_file():
        raise RuntimeError(
            f"FASTRTPS_DEFAULT_PROFILES_FILE does not exist: {profile_path}"
        )
    environment["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile_path)
    return environment


def _create_jetson_actions(context):
    environment_file = LaunchConfiguration(
        "network_environment_file"
    ).perform(context)
    environment = _load_network_environment(environment_file)
    description_file = Path(
        LaunchConfiguration("robot_description_file").perform(context)
    ).expanduser().resolve()
    if not description_file.is_file():
        raise RuntimeError(
            f"robot_description_file does not exist: {description_file}"
        )
    robot_description = description_file.read_text(encoding="utf-8")
    actions = [
        SetEnvironmentVariable(name, value)
        for name, value in environment.items()
    ]
    actions.extend(
        [
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
                parameters=[
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
                        "auto_configure": LaunchConfiguration(
                            "auto_configure"
                        ),
                        "auto_activate": LaunchConfiguration("auto_activate"),
                    }
                ],
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Create a Jetson deployment with no implicit DDS or hardware defaults."""
    description_share = get_package_share_directory("myarm_m750_description")
    default_description = os.path.join(
        description_share,
        "urdf",
        "generated",
        "myarm_m750_full.urdf",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "core_config_file",
                description="Absolute validated Python Core v0.2 profile.",
            ),
            DeclareLaunchArgument(
                "network_environment_file",
                description="Validated Fast DDS environment artifact.",
            ),
            DeclareLaunchArgument(
                "robot_description_file", default_value=default_description
            ),
            DeclareLaunchArgument("use_real_hardware", default_value="false"),
            DeclareLaunchArgument(
                "enable_command_interfaces", default_value="false"
            ),
            DeclareLaunchArgument("auto_configure", default_value="true"),
            DeclareLaunchArgument("auto_activate", default_value="true"),
            OpaqueFunction(function=_create_jetson_actions),
        ]
    )
