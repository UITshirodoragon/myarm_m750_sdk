"""Minimal non-ROS smoke test for the public SDK boundary."""

from __future__ import annotations

import argparse

from myarm_m750_core import RobotSession


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    arguments = parser.parse_args()

    with RobotSession.from_config(arguments.config) as robot:
        initial = robot.get_state()
        print("initial_joint_rad:", initial.position_rad)
        result = robot.move_joints(
            target=[0.20, -0.20, 0.15, 0.10, -0.10, 0.15],
            duration_s=3.0,
        )
        print("command:", result.status.value, result.message, result.command_id)
        final = robot.get_state()
        print("final_joint_rad:", final.position_rad)
        print("tool0_pose:", robot.compute_fk(final.position_rad))


if __name__ == "__main__":
    main()
