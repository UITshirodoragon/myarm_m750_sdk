"""MoveIt mock execution routed through the SDK trajectory action."""

from __future__ import annotations

from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from myarm_m750_moveit_config.plan_only import move_group_nodes


def _launch_setup(context: LaunchContext) -> List[object]:
    core_config_file = LaunchConfiguration("core_config_file").perform(context)
    if not Path(core_config_file).expanduser().is_file():
        raise ValueError("core_config_file must point to a readable mock SDK YAML.")
    driver_launch = (
        Path(get_package_share_directory("myarm_m750_driver"))
        / "launch"
        / "driver.launch.py"
    )
    nodes = move_group_nodes(context, allow_trajectory_execution=True)
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver_launch)),
            launch_arguments={
                "core_config_file": str(Path(core_config_file).expanduser().resolve()),
                "use_real_hardware": "false",
                "enable_command_interfaces": "true",
            }.items(),
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("core_config_file"),
            DeclareLaunchArgument("model_variant", default_value="lightweight"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
