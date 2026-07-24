"""Small B1/T1 harness that emits machine-readable JSON lines."""

from __future__ import annotations

import argparse
import json
import time

import yaml

from myarm_m750_core import RobotSession


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-config", required=True)
    parser.add_argument("--benchmark-config", required=True)
    arguments = parser.parse_args()
    with open(arguments.benchmark_config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["benchmark"]

    with RobotSession.from_config(arguments.sdk_config) as robot:
        for repetition in range(int(config["repetitions"])):
            for waypoint_index, waypoint in enumerate(config["waypoints_rad"]):
                start_s = time.monotonic()
                result = robot.move_joints(waypoint, float(config["duration_s"]))
                measured = robot.get_state()
                payload = {
                    "repetition": repetition,
                    "waypoint_index": waypoint_index,
                    "status": result.status.value,
                    "command_id": result.command_id,
                    "elapsed_s": time.monotonic() - start_s,
                    "target_rad": waypoint,
                    "measured_rad": measured.position_rad,
                }
                print(json.dumps(payload))


if __name__ == "__main__":
    main()
