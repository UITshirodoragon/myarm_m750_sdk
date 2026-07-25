"""Read-only Host role: RViz2, diagnostics observation, and network probe."""

import os
from dataclasses import asdict

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from myarm_m750_visualization.network_contract import (
    NetworkContract,
    environment_for_contract,
    load_network_contract,
    validate_local_interface,
)


def _network_budget_parameters(contract: NetworkContract):
    """Map the validated deployment budget onto probe parameter names."""
    return asdict(contract.budget)


def _create_role_actions(context):
    package_share = get_package_share_directory("myarm_m750_visualization")
    config_file = LaunchConfiguration("network_config_file").perform(context)
    profile_file = LaunchConfiguration("fastdds_profile_file").perform(context)
    if not os.path.isfile(profile_file):
        raise RuntimeError(
            f"fastdds_profile_file does not exist: {profile_file}"
        )
    contract = load_network_contract(config_file, expected_role="host")
    if (
        LaunchConfiguration("check_local_interface").perform(context).lower()
        == "true"
    ):
        validate_local_interface(contract)
    actions = [
        SetEnvironmentVariable(name, value)
        for name, value in environment_for_contract(
            contract, profile_file
        ).items()
    ]
    probe_parameters = os.path.join(
        package_share, "config", "network_probe.yaml"
    )
    report_json_file = LaunchConfiguration("report_json_file").perform(context)
    report_csv_file = LaunchConfiguration("report_csv_file").perform(context)
    report_interval_s = float(
        LaunchConfiguration("report_interval_s").perform(context)
    )
    clock_offset_source = LaunchConfiguration(
        "clock_offset_source"
    ).perform(context)
    measured_clock_offset_ms = float(
        LaunchConfiguration("measured_clock_offset_ms").perform(context)
    )
    require_clock_offset_measurement = (
        LaunchConfiguration("require_clock_offset_measurement")
        .perform(context)
        .lower()
        == "true"
    )
    actions.extend(
        [
            Node(
                package="myarm_m750_visualization",
                executable="network_probe",
                name="myarm_m750_network_probe",
                output="screen",
                parameters=[
                    # The generic YAML supplies standalone-node defaults.  The
                    # machine contract is the runtime owner for every budget.
                    probe_parameters,
                    _network_budget_parameters(contract),
                    {
                        "report_json_file": report_json_file,
                        "report_csv_file": report_csv_file,
                        "report_interval_s": report_interval_s,
                        "clock_offset_source": clock_offset_source,
                        "measured_clock_offset_ms": measured_clock_offset_ms,
                        "require_clock_offset_measurement": (
                            require_clock_offset_measurement
                        ),
                    },
                ],
                condition=IfCondition(
                    LaunchConfiguration("enable_network_probe")
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_myarm_m750_host",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        package_share, "rviz", "robot_host.rviz"
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Create an explicit read-only Host deployment."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "network_config_file",
                description="Absolute Host network contract YAML.",
            ),
            DeclareLaunchArgument(
                "fastdds_profile_file",
                description="Generated machine-specific Fast DDS XML profile.",
            ),
            DeclareLaunchArgument(
                "check_local_interface", default_value="true"
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "enable_network_probe", default_value="true"
            ),
            DeclareLaunchArgument("report_json_file", default_value=""),
            DeclareLaunchArgument("report_csv_file", default_value=""),
            DeclareLaunchArgument("report_interval_s", default_value="5.0"),
            DeclareLaunchArgument("clock_offset_source", default_value=""),
            DeclareLaunchArgument(
                "measured_clock_offset_ms",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "require_clock_offset_measurement",
                default_value="true",
            ),
            OpaqueFunction(function=_create_role_actions),
        ]
    )
