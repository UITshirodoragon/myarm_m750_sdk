"""Demonstrate software FK and numerical IK without opening hardware."""

from __future__ import annotations

import argparse

from myarm_m750_core import MotionProfile, RobotSessionBuilder


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    arguments = parser.parse_args()
    with RobotSessionBuilder.from_file(arguments.config).build() as robot:
        target_joint_rad = [0.2, -0.3, 0.4, 0.2, -0.2, 0.1]
        target_pose = robot.compute_fk(target_joint_rad)
        print("FK target:", target_pose)
        result = robot.move_pose(
            target_pose,
            MotionProfile(duration_s=4.0),
            seed_joint_position_rad=[0.23, -0.27, 0.43, 0.23, -0.17, 0.13],
        )
        print("IK + execution:", result.status.value, result.message)
        print("measured_joint_rad:", robot.read_joint_state().position_rad)


if __name__ == "__main__":
    main()
