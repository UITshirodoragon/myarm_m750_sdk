"""Shared manual MoveIt parameter composition for ROS 2 Foxy."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from myarm_m750_moveit_config.model_catalog import load_model_catalog


def _load_yaml(package_name: str, relative_path: str) -> Dict[str, object]:
    path = Path(get_package_share_directory(package_name)) / relative_path
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    return loaded if isinstance(loaded, dict) else {}


def move_group_nodes(
    context: LaunchContext, allow_trajectory_execution: bool
) -> List[Node]:
    """Build robot_state_publisher and move_group with manual Foxy parameters."""
    variant = LaunchConfiguration("model_variant").perform(context)
    description_share = Path(get_package_share_directory("myarm_m750_description"))
    model_catalog = load_model_catalog(description_share)
    model_variant = model_catalog.planning_variant(variant)
    moveit_share = Path(get_package_share_directory("myarm_m750_moveit_config"))
    robot_description = model_variant.path.read_text(encoding="utf-8")
    robot_description_semantic = (
        moveit_share / "config" / "myarm_m750.srdf"
    ).read_text(encoding="utf-8")
    ompl_pipeline = _load_yaml(
        "myarm_m750_moveit_config", "config/planning_pipeline.yaml"
    )
    ompl_pipeline.update(
        _load_yaml("myarm_m750_moveit_config", "config/ompl_planning.yaml")
    )
    parameters = [
        {"robot_description": robot_description},
        {"robot_description_semantic": robot_description_semantic},
        {
            "robot_description_kinematics": _load_yaml(
                "myarm_m750_moveit_config", "config/kinematics.yaml"
            )
        },
        {
            "robot_description_planning": _load_yaml(
                "myarm_m750_moveit_config", "config/joint_limits.yaml"
            )
        },
        # Foxy resolves private MoveGroup parameters under ``move_group.*``.
        # The newer ``planning_pipelines`` parameter tree is not backported.
        {"move_group": ompl_pipeline},
        {
            "allow_trajectory_execution": allow_trajectory_execution,
            "myarm_m750_model_contract_sha256": (
                model_catalog.kinematic_contract_sha256
            ),
            "myarm_m750_collision_provenance": (
                model_catalog.collision_provenance
            ),
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
    ]
    if allow_trajectory_execution:
        parameters.append(
            _load_yaml(
                "myarm_m750_moveit_config",
                "config/moveit_controllers.yaml",
            )
        )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="myarm_m750_moveit_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=parameters,
        ),
    ]
