"""Minimal non-ROS smoke test for the public SDK boundary."""

from __future__ import annotations

import argparse

from myarm_m750_core import MotionProfile, RobotSessionBuilder


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    arguments = parser.parse_args()

    with RobotSessionBuilder.from_file(arguments.config).build() as robot:
        initial = robot.read_joint_state()
        print("initial_joint_rad:", initial.position_rad)
        result = robot.move_joints(
            [0.20, -0.20, 0.15, 0.10, -0.10, 0.15],
            MotionProfile(duration_s=3.0),
        )
        print("command:", result.status.value, result.message, result.command_id)
        final = robot.read_joint_state()
        print("final_joint_rad:", final.position_rad)
        print("tool0_pose:", robot.compute_fk(final.position_rad))


if __name__ == "__main__":
    main()
