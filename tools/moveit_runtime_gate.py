#!/usr/bin/env python3
"""Run bounded, reproducible MoveIt Foxy plan and mock-execution gates."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROBE = (
    ROOT
    / "ros2"
    / "src"
    / "myarm_m750_moveit_config"
    / "test"
    / "moveit_runtime_probe.py"
)


class GateFailure(RuntimeError):
    """A MoveIt process or machine-readable contract failed its gate."""


class ManagedLaunch:
    """Own one ROS launch process group and its output log."""

    _FORBIDDEN_SHUTDOWN_MARKERS = (
        "KeyboardInterrupt",
        "failed to terminate",
        "exit code -6",
        "exit code -11",
        "exit code -15",
    )

    def __init__(
        self,
        package: str,
        launch_file: str,
        arguments: Sequence[str],
        log_file: Path,
        environment: Mapping[str, str],
    ) -> None:
        command = ["ros2", "launch", package, launch_file]
        command.extend(arguments)
        log_file.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = log_file
        self._stream = log_file.open("w", encoding="utf-8")
        self._process = subprocess.Popen(
            command,
            env=dict(environment),
            stdout=self._stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def assert_running(self, context: str) -> None:
        """Fail with recent launch output when launch exits unexpectedly."""
        return_code = self._process.poll()
        if return_code is None:
            return
        self._stream.flush()
        raise GateFailure(
            f"{context}: launch exited with code {return_code}.\n"
            f"{self._log_tail()}"
        )

    def stop(self, timeout_s: float) -> Dict[str, object]:
        """Let ros2 launch stop its children, then require a clean deadline."""
        started_s = time.monotonic()
        forced_signal = ""
        try:
            if self._process.poll() is None:
                # Signal only the launch parent. Sending SIGINT to the whole
                # group races launch's own child signaling on Foxy and can
                # interrupt rclpy cleanup twice.
                self._process.send_signal(signal.SIGINT)
                try:
                    self._process.wait(timeout=timeout_s)
                except subprocess.TimeoutExpired:
                    forced_signal = "SIGTERM"
                    self._signal_group(signal.SIGTERM)
                    try:
                        self._process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        forced_signal = "SIGKILL"
                        self._signal_group(signal.SIGKILL)
                        self._process.wait(timeout=2.0)
            return_code = self._process.returncode
        finally:
            self._stream.close()
        elapsed_s = time.monotonic() - started_s
        if forced_signal:
            raise GateFailure(
                "MoveIt launch did not stop cleanly after process-group "
                f"SIGINT; required {forced_signal} after {elapsed_s:.3f}s.\n"
                f"{self._log_tail()}"
            )
        self._assert_clean_shutdown_log()
        return {
            "elapsed_s": elapsed_s,
            "signal": "SIGINT",
            "return_code": return_code,
        }

    def _signal_group(self, requested_signal: signal.Signals) -> None:
        try:
            os.killpg(self._process.pid, requested_signal)
        except ProcessLookupError:
            return

    def _assert_clean_shutdown_log(self) -> None:
        output = self._log_file.read_text(
            encoding="utf-8",
            errors="replace",
        )
        matched = [
            marker
            for marker in self._FORBIDDEN_SHUTDOWN_MARKERS
            if marker in output
        ]
        if matched:
            raise GateFailure(
                "MoveIt launch reported an unclean child shutdown "
                f"({', '.join(matched)}).\n{output[-6000:]}"
            )

    def _log_tail(self) -> str:
        try:
            return self._log_file.read_text(
                encoding="utf-8",
                errors="replace",
            )[-6000:]
        except OSError:
            return "<launch log unavailable>"


def _run_probe(
    probe_path: Path,
    mode: str,
    probe_timeout_s: float,
    scenario_timeout_s: float,
    log_file: Path,
    environment: Mapping[str, str],
) -> Tuple[Dict[str, object], float]:
    command = [
        sys.executable,
        str(probe_path),
        "--mode",
        mode,
        "--timeout-s",
        str(probe_timeout_s),
    ]
    started_s = time.monotonic()
    process = subprocess.Popen(
        command,
        env=dict(environment),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        output, _ = process.communicate(timeout=scenario_timeout_s)
    except subprocess.TimeoutExpired as error:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            output, _ = process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=2.0)
        log_file.write_text(output, encoding="utf-8")
        raise GateFailure(
            f"{mode} probe exceeded {scenario_timeout_s:.1f}s.\n"
            f"{output[-6000:]}"
        ) from error
    elapsed_s = time.monotonic() - started_s
    log_file.write_text(output, encoding="utf-8")
    if process.returncode != 0:
        raise GateFailure(
            f"{mode} probe exited with code {process.returncode}.\n"
            f"{output[-6000:]}"
        )
    return _parse_last_json_object(output, mode), elapsed_s


def _parse_last_json_object(output: str, mode: str) -> Dict[str, object]:
    for line in reversed(output.splitlines()):
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed
    raise GateFailure(
        f"{mode} probe emitted no JSON object.\n{output[-6000:]}"
    )


def _validate_common_report(
    report: Mapping[str, object],
    expected_mode: str,
) -> None:
    if report.get("mode") != expected_mode:
        raise GateFailure(
            f"{expected_mode} report has wrong mode: {report.get('mode')!r}."
        )
    if report.get("planning_error_code") != MoveItErrorCodes.SUCCESS:
        raise GateFailure(
            f"{expected_mode} planning did not return MoveIt SUCCESS."
        )
    point_count = report.get("planning_point_count")
    if (
        not isinstance(point_count, int)
        or isinstance(point_count, bool)
        or point_count < 2
    ):
        raise GateFailure(
            f"{expected_mode} returned an invalid point count: "
            f"{point_count!r}."
        )
    raw_times = report.get("planning_times_s")
    if not isinstance(raw_times, list) or len(raw_times) != point_count:
        raise GateFailure(
            f"{expected_mode} returned an invalid trajectory time vector."
        )
    times_s = []
    for raw_time in raw_times:
        numeric_time = isinstance(raw_time, (int, float))
        if isinstance(raw_time, bool) or not numeric_time:
            raise GateFailure(
                f"{expected_mode} returned a non-numeric trajectory time."
            )
        time_s = float(raw_time)
        if not math.isfinite(time_s) or time_s < 0.0:
            raise GateFailure(
                f"{expected_mode} returned an invalid trajectory time."
            )
        times_s.append(time_s)
    if any(
        current_s <= previous_s
        for previous_s, current_s in zip(times_s, times_s[1:])
    ):
        raise GateFailure(
            f"{expected_mode} trajectory times are not strictly increasing."
        )
    if report.get("collision_state_valid") is not False:
        raise GateFailure(
            f"{expected_mode} blocking object did not invalidate the state."
        )
    if (
        report.get("collision_planning_error_code")
        == MoveItErrorCodes.SUCCESS
    ):
        raise GateFailure(
            f"{expected_mode} planned through the blocking collision object."
        )
    if any(key.startswith("cancel_") for key in report):
        raise GateFailure(
            f"{expected_mode} report mixed cancellation into a passing gate."
        )


def _validate_report(
    report: Mapping[str, object],
    expected_mode: str,
) -> None:
    _validate_common_report(report, expected_mode)
    if expected_mode == "plan-only":
        if any(key.startswith("execution_") for key in report):
            raise GateFailure(
                "plan-only report unexpectedly contains execution evidence."
            )
        return
    if report.get("execution_status") != GoalStatus.STATUS_SUCCEEDED:
        raise GateFailure("mock-execution did not reach SUCCEEDED.")
    if report.get("execution_error_code") != MoveItErrorCodes.SUCCESS:
        raise GateFailure(
            "mock-execution did not return MoveIt SUCCESS."
        )


def _scenario_environment(
    base_environment: Mapping[str, str],
    domain_id: int,
    ros_log_directory: Path,
) -> Dict[str, str]:
    environment = dict(base_environment)
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    environment["ROS_LOCALHOST_ONLY"] = "1"
    environment["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    environment["ROS_LOG_DIR"] = str(ros_log_directory)
    environment["PYTHONNOUSERSITE"] = "1"
    return environment


def _run_scenario(
    *,
    mode: str,
    launch_file: str,
    launch_arguments: Sequence[str],
    domain_id: int,
    probe_path: Path,
    log_directory: Path,
    probe_timeout_s: float,
    scenario_timeout_s: float,
    shutdown_timeout_s: float,
) -> Dict[str, object]:
    scenario_log_directory = log_directory / mode
    ros_log_directory = scenario_log_directory / "ros"
    ros_log_directory.mkdir(parents=True, exist_ok=True)
    environment = _scenario_environment(
        os.environ,
        domain_id,
        ros_log_directory,
    )
    launch = ManagedLaunch(
        "myarm_m750_moveit_config",
        launch_file,
        launch_arguments,
        scenario_log_directory / "launch.log",
        environment,
    )
    operation_error = None  # type: Optional[BaseException]
    report = None  # type: Optional[Dict[str, object]]
    probe_elapsed_s = 0.0
    try:
        launch.assert_running(f"{mode} startup")
        report, probe_elapsed_s = _run_probe(
            probe_path,
            mode,
            probe_timeout_s,
            scenario_timeout_s,
            scenario_log_directory / "probe.log",
            environment,
        )
        launch.assert_running(f"{mode} completion")
        _validate_report(report, mode)
    except BaseException as error:
        # Always reap the launch group, including on Ctrl-C.
        operation_error = error

    try:
        shutdown = launch.stop(shutdown_timeout_s)
    except Exception as cleanup_error:
        if operation_error is not None:
            raise GateFailure(
                f"{operation_error}\nCleanup also failed: {cleanup_error}"
            ) from operation_error
        raise
    if operation_error is not None:
        raise operation_error
    if report is None:
        raise GateFailure(f"{mode} produced no report.")
    return {
        "domain_id": domain_id,
        "probe_elapsed_s": probe_elapsed_s,
        "launch_shutdown": shutdown,
        "probe": report,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run passing MoveIt gates and print one machine-readable report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core-config", required=True, type=Path)
    parser.add_argument("--probe", type=Path, default=DEFAULT_PROBE)
    parser.add_argument("--log-directory", required=True, type=Path)
    parser.add_argument("--probe-timeout-s", type=float, default=20.0)
    parser.add_argument("--scenario-timeout-s", type=float, default=90.0)
    parser.add_argument("--shutdown-timeout-s", type=float, default=10.0)
    parser.add_argument("--domain-id-base", type=int, default=52)
    arguments = parser.parse_args(argv)
    for name, timeout_s in (
        ("--probe-timeout-s", arguments.probe_timeout_s),
        ("--scenario-timeout-s", arguments.scenario_timeout_s),
        ("--shutdown-timeout-s", arguments.shutdown_timeout_s),
    ):
        if timeout_s <= 0.0:
            raise ValueError(f"{name} must be positive.")
    if arguments.domain_id_base < 0 or arguments.domain_id_base > 231:
        raise ValueError("--domain-id-base must be in [0, 231].")
    core_config = arguments.core_config.resolve()
    probe_path = arguments.probe.resolve()
    if not core_config.is_file():
        raise ValueError(f"Core config is not readable: {core_config}.")
    if not probe_path.is_file():
        raise ValueError(f"MoveIt probe is not readable: {probe_path}.")
    log_directory = arguments.log_directory.resolve()
    log_directory.mkdir(parents=True, exist_ok=True)

    plan_only = _run_scenario(
        mode="plan-only",
        launch_file="plan_only.launch.py",
        launch_arguments=("model_variant:=lightweight",),
        domain_id=arguments.domain_id_base,
        probe_path=probe_path,
        log_directory=log_directory,
        probe_timeout_s=arguments.probe_timeout_s,
        scenario_timeout_s=arguments.scenario_timeout_s,
        shutdown_timeout_s=arguments.shutdown_timeout_s,
    )
    mock_execution = _run_scenario(
        mode="mock-execution",
        launch_file="mock_execution.launch.py",
        launch_arguments=(
            f"core_config_file:={core_config}",
            "model_variant:=lightweight",
        ),
        domain_id=arguments.domain_id_base + 1,
        probe_path=probe_path,
        log_directory=log_directory,
        probe_timeout_s=arguments.probe_timeout_s,
        scenario_timeout_s=arguments.scenario_timeout_s,
        shutdown_timeout_s=arguments.shutdown_timeout_s,
    )
    report = {
        "gate": "moveit_foxy_runtime",
        "rmw_implementation": "rmw_fastrtps_cpp",
        "model_variant": "lightweight",
        "core_config": str(core_config),
        "probe_source": str(probe_path),
        "plan_only": plan_only,
        "mock_execution": mock_execution,
        "mock_cancel": {
            "executed": False,
            "classification": "expected_foxy_blocker",
        },
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
