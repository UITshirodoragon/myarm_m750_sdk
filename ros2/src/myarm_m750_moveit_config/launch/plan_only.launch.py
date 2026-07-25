"""Headless MoveIt planning against the canonical generated model."""

from __future__ import annotations

from typing import List

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from myarm_m750_moveit_config.plan_only import move_group_nodes


def _launch_setup(context: LaunchContext) -> List[object]:
    return move_group_nodes(context, allow_trajectory_execution=False)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_variant", default_value="lightweight"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
