"""Validated Fast DDS WLAN configuration and profile generation."""

from __future__ import annotations

import argparse
import fcntl
import ipaddress
import json
import math
import socket
import struct
import xml.etree.ElementTree as element_tree
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml  # type: ignore[import-untyped]

_SUPPORTED_DISCOVERY_MODES = ("multicast", "peer", "discovery_server")
_EXPECTED_RMW = "rmw_fastrtps_cpp"


@dataclass(frozen=True)
class NetworkBudget:
    """Provisional observation-channel acceptance limits."""

    minimum_joint_state_rate_hz: float
    maximum_p95_age_ms: float
    maximum_p99_age_ms: float
    maximum_gap_s: float
    maximum_reconnect_s: float
    maximum_clock_offset_ms: float
    maximum_control_bandwidth_mbit_s: float


@dataclass(frozen=True)
class NetworkContract:
    """One explicit machine-specific Fast DDS deployment contract."""

    schema_version: int
    role: str
    rmw_implementation: str
    ros_domain_id: int
    wlan_interface: str
    interface_address: str
    discovery_mode: str
    peer_addresses: Tuple[str, ...]
    discovery_server: Optional[str]
    budget: NetworkBudget
    source_path: Path


def load_network_contract(config_file: str, expected_role: str) -> NetworkContract:
    """Load a strict network contract from YAML.

    Unknown keys are rejected so misspelled safety/deployment settings never
    silently fall back to DDS defaults.
    """
    source_path = Path(config_file).expanduser().resolve()
    with source_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    root = _require_mapping(document, "document")
    _reject_unknown(root, {"schema_version", "network", "budget"}, "document")
    if root.get("schema_version") != 1:
        raise ValueError("schema_version must equal 1.")

    network = _require_mapping(root.get("network"), "network")
    _reject_unknown(
        network,
        {
            "role",
            "rmw_implementation",
            "ros_domain_id",
            "wlan_interface",
            "interface_address",
            "discovery",
        },
        "network",
    )
    role = _required_string(network, "role")
    if role != expected_role:
        raise ValueError(
            f"network.role='{role}' does not match expected role '{expected_role}'."
        )
    rmw_implementation = _required_string(network, "rmw_implementation")
    if rmw_implementation != _EXPECTED_RMW:
        raise ValueError(
            f"rmw_implementation must be '{_EXPECTED_RMW}' for this release."
        )
    ros_domain_id = int(network.get("ros_domain_id", -1))
    if ros_domain_id < 0 or ros_domain_id > 232:
        raise ValueError("network.ros_domain_id must be in [0, 232].")
    wlan_interface = _required_string(network, "wlan_interface")
    interface_address = _validate_ipv4(
        _required_string(network, "interface_address"),
        "network.interface_address",
    )

    discovery = _require_mapping(network.get("discovery"), "network.discovery")
    _reject_unknown(
        discovery, {"mode", "peer_addresses", "server"}, "network.discovery"
    )
    discovery_mode = _required_string(discovery, "mode")
    if discovery_mode not in _SUPPORTED_DISCOVERY_MODES:
        raise ValueError(
            f"network.discovery.mode must be one of {_SUPPORTED_DISCOVERY_MODES}."
        )
    peer_addresses = tuple(
        _validate_ipv4(str(address), "network.discovery.peer_addresses")
        for address in _require_list(
            discovery.get("peer_addresses", []),
            "network.discovery.peer_addresses",
        )
    )
    if discovery_mode == "peer" and not peer_addresses:
        raise ValueError("peer discovery requires at least one peer address.")
    server_value = discovery.get("server")
    discovery_server = (
        None if server_value in (None, "") else _validate_server(str(server_value))
    )
    if discovery_mode == "discovery_server" and discovery_server is None:
        raise ValueError("discovery_server mode requires discovery.server.")

    budget_data = _require_mapping(root.get("budget"), "budget")
    expected_budget_keys = {
        "minimum_joint_state_rate_hz",
        "maximum_p95_age_ms",
        "maximum_p99_age_ms",
        "maximum_gap_s",
        "maximum_reconnect_s",
        "maximum_clock_offset_ms",
        "maximum_control_bandwidth_mbit_s",
    }
    _reject_unknown(budget_data, expected_budget_keys, "budget")
    if set(budget_data) != expected_budget_keys:
        missing = sorted(expected_budget_keys - set(budget_data))
        raise ValueError(f"budget is missing required fields: {missing}.")
    budget_values = {
        key: _positive_float(budget_data[key], f"budget.{key}")
        for key in expected_budget_keys
    }
    return NetworkContract(
        schema_version=1,
        role=role,
        rmw_implementation=rmw_implementation,
        ros_domain_id=ros_domain_id,
        wlan_interface=wlan_interface,
        interface_address=interface_address,
        discovery_mode=discovery_mode,
        peer_addresses=peer_addresses,
        discovery_server=discovery_server,
        budget=NetworkBudget(**budget_values),
        source_path=source_path,
    )


def validate_local_interface(contract: NetworkContract) -> None:
    """Verify the declared interface and IPv4 address exist on this machine."""
    interface_names = {name for _, name in socket.if_nameindex()}
    if contract.wlan_interface not in interface_names:
        raise ValueError(
            f"Interface '{contract.wlan_interface}' does not exist on this machine."
        )
    request = struct.pack(
        "256s", contract.wlan_interface[:15].encode("utf-8")
    )
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as interface_socket:
        try:
            response = fcntl.ioctl(interface_socket.fileno(), 0x8915, request)
        except OSError as error:
            raise ValueError(
                f"Cannot read IPv4 address for {contract.wlan_interface}: {error}"
            ) from error
    assigned_address = socket.inet_ntoa(response[20:24])
    if contract.interface_address != assigned_address:
        raise ValueError(
            f"Address '{contract.interface_address}' does not match "
            f"{contract.wlan_interface} address '{assigned_address}'."
        )


def render_fastdds_profile(contract: NetworkContract) -> str:
    """Render a machine-specific Fast DDS UDPv4 profile."""
    profiles = element_tree.Element(
        "profiles",
        {"xmlns": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"},
    )
    transports = element_tree.SubElement(profiles, "transport_descriptors")
    descriptor = element_tree.SubElement(
        transports, "transport_descriptor"
    )
    element_tree.SubElement(descriptor, "transport_id").text = (
        "myarm_m750_wlan_udp"
    )
    element_tree.SubElement(descriptor, "type").text = "UDPv4"
    whitelist = element_tree.SubElement(descriptor, "interfaceWhiteList")
    element_tree.SubElement(whitelist, "address").text = (
        contract.interface_address
    )

    participant = element_tree.SubElement(
        profiles,
        "participant",
        {"profile_name": "myarm_m750_wlan", "is_default_profile": "true"},
    )
    rtps = element_tree.SubElement(participant, "rtps")
    user_transports = element_tree.SubElement(rtps, "userTransports")
    element_tree.SubElement(user_transports, "transport_id").text = (
        "myarm_m750_wlan_udp"
    )
    element_tree.SubElement(rtps, "useBuiltinTransports").text = "false"
    builtin = element_tree.SubElement(rtps, "builtin")
    if contract.discovery_mode == "peer":
        # Fast DDS 2.1 supports this explicit unicast-only discovery switch;
        # emitting it prevents peer mode from inheriting multicast behavior.
        element_tree.SubElement(
            builtin, "avoid_builtin_multicast"
        ).text = "true"
        peers = element_tree.SubElement(builtin, "initialPeersList")
        for peer_address in contract.peer_addresses:
            locator = element_tree.SubElement(peers, "locator")
            udp = element_tree.SubElement(locator, "udpv4")
            element_tree.SubElement(udp, "address").text = peer_address
    elif contract.discovery_mode == "multicast":
        element_tree.SubElement(
            builtin, "avoid_builtin_multicast"
        ).text = "false"
    xml_body = element_tree.tostring(profiles, encoding="unicode")
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + xml_body + "\n"


def environment_for_contract(
    contract: NetworkContract, profile_file: str
) -> Dict[str, str]:
    """Return the process environment required by ROS 2 Foxy/Fast DDS."""
    environment = {
        "MYARM_M750_ROLE": contract.role,
        "MYARM_M750_WLAN_INTERFACE": contract.wlan_interface,
        "RMW_IMPLEMENTATION": contract.rmw_implementation,
        "ROS_DOMAIN_ID": str(contract.ros_domain_id),
        "FASTRTPS_DEFAULT_PROFILES_FILE": str(Path(profile_file).resolve()),
    }
    if contract.discovery_server is not None:
        environment["ROS_DISCOVERY_SERVER"] = contract.discovery_server
    return environment


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Validate config and optionally generate Fast DDS/env artifacts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--role", choices=("jetson", "host"), required=True)
    parser.add_argument("--check-interface", action="store_true")
    parser.add_argument("--profile-output")
    parser.add_argument("--environment-output")
    parser.add_argument("--json", action="store_true")
    arguments = parser.parse_args(argv)

    contract = load_network_contract(arguments.config, arguments.role)
    if arguments.check_interface:
        validate_local_interface(contract)
    profile_file = arguments.profile_output
    if profile_file:
        profile_path = Path(profile_file).expanduser().resolve()
        profile_path.parent.mkdir(parents=True, exist_ok=True)
        profile_path.write_text(render_fastdds_profile(contract), encoding="utf-8")
    if arguments.environment_output:
        if not profile_file:
            raise ValueError("--environment-output requires --profile-output.")
        environment = environment_for_contract(contract, profile_file)
        environment_path = Path(arguments.environment_output).expanduser().resolve()
        environment_path.parent.mkdir(parents=True, exist_ok=True)
        environment_path.write_text(
            "".join(
                f"{key}={value}\n"
                for key, value in sorted(environment.items())
            ),
            encoding="utf-8",
        )
    if arguments.json:
        serialized = asdict(contract)
        serialized["source_path"] = str(contract.source_path)
        print(json.dumps(serialized, indent=2, sort_keys=True))
    else:
        print(
            f"valid role={contract.role} domain={contract.ros_domain_id} "
            f"interface={contract.wlan_interface} "
            f"discovery={contract.discovery_mode}"
        )
    return 0


def _require_mapping(value: Any, field_name: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a mapping.")
    return value


def _require_list(value: Any, field_name: str) -> List[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{field_name} must be a list.")
    return value


def _reject_unknown(
    mapping: Dict[str, Any], expected: set, field_name: str
) -> None:
    unknown = sorted(set(mapping) - expected)
    if unknown:
        raise ValueError(
            f"{field_name} contains unknown fields: {unknown}."
        )


def _required_string(mapping: Dict[str, Any], key: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string.")
    return value.strip()


def _validate_ipv4(value: str, field_name: str) -> str:
    try:
        address = ipaddress.ip_address(value)
    except ValueError as error:
        raise ValueError(f"{field_name} must be an IPv4 address.") from error
    if address.version != 4:
        raise ValueError(f"{field_name} must be an IPv4 address.")
    return str(address)


def _validate_server(value: str) -> str:
    host, separator, port_text = value.rpartition(":")
    if not separator:
        raise ValueError("network.discovery.server must use IPv4:port.")
    _validate_ipv4(host, "network.discovery.server")
    port = int(port_text)
    if port <= 0 or port > 65535:
        raise ValueError("network.discovery.server port must be in [1, 65535].")
    return f"{host}:{port}"


def _positive_float(value: Any, field_name: str) -> float:
    converted = float(value)
    if not math.isfinite(converted) or converted <= 0.0:
        raise ValueError(f"{field_name} must be finite and positive.")
    return converted


if __name__ == "__main__":
    raise SystemExit(main())
