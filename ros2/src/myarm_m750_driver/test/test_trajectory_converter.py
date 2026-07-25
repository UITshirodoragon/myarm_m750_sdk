"""Regression tests for the pure FollowJointTrajectory converter."""

import math
import unittest
from types import SimpleNamespace

from myarm_m750_driver.contracts import GoalConversionError, TrajectoryErrorCode
from myarm_m750_driver.trajectory_converter import (
    convert_follow_joint_trajectory_goal,
    violated_joint_names,
)

JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")


def _duration(seconds=0, nanoseconds=0):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def _point(positions, seconds, velocities=(), accelerations=(), effort=()):
    return SimpleNamespace(
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
        effort=effort,
        time_from_start=_duration(seconds),
    )


def _goal(
    joint_names=JOINT_NAMES,
    points=None,
    stamp=None,
    path_tolerance=(),
    goal_tolerance=(),
):
    if points is None:
        points = [_point((0.0,) * 6, 0), _point((1.0,) * 6, 1)]
    if stamp is None:
        stamp = _duration()
    return SimpleNamespace(
        trajectory=SimpleNamespace(
            joint_names=joint_names,
            points=points,
            header=SimpleNamespace(stamp=stamp),
        ),
        path_tolerance=path_tolerance,
        goal_tolerance=goal_tolerance,
        goal_time_tolerance=_duration(),
    )


def _convert(goal):
    return convert_follow_joint_trajectory_goal(
        goal,
        canonical_joint_names=JOINT_NAMES,
        maximum_trajectory_points=1000,
        now_ros_s=100.0,
        default_path_tolerance_rad=0.2,
        default_goal_tolerance_rad=0.05,
        default_goal_time_tolerance_s=0.5,
        old_header_tolerance_s=0.5,
    )


class TrajectoryConverterTest(unittest.TestCase):
    """Exercise joint, time, derivative, and tolerance boundary semantics."""

    def test_accepts_permutation_and_reorders_vectors(self) -> None:
        accepted = _convert(
            _goal(
                joint_names=tuple(reversed(JOINT_NAMES)),
                points=[
                    _point(
                        positions=(6, 5, 4, 3, 2, 1),
                        velocities=(60, 50, 40, 30, 20, 10),
                        accelerations=(600, 500, 400, 300, 200, 100),
                        seconds=0,
                    )
                ],
            )
        )

        point = accepted.trajectory.points[0]
        self.assertEqual(point.position_rad, (1, 2, 3, 4, 5, 6))
        self.assertEqual(point.velocity_rad_s, (10, 20, 30, 40, 50, 60))
        self.assertEqual(
            point.acceleration_rad_s2, (100, 200, 300, 400, 500, 600)
        )

    def test_rejects_invalid_joint_sets(self) -> None:
        invalid_sets = (
            ("j1", "j1", "j3", "j4", "j5", "j6"),
            ("j1", "j2", "j3", "j4", "j5", "unknown"),
            ("j1", "j2", "j3", "j4", "j5"),
        )
        for names in invalid_sets:
            with self.subTest(names=names):
                with self.assertRaises(GoalConversionError) as raised:
                    _convert(_goal(joint_names=names))
                self.assertIs(
                    raised.exception.code, TrajectoryErrorCode.INVALID_JOINTS
                )

    def test_rejects_non_finite_and_non_increasing_points(self) -> None:
        with self.assertRaises(GoalConversionError):
            _convert(_goal(points=[_point((math.nan,) * 6, 0)]))
        with self.assertRaises(GoalConversionError):
            _convert(
                _goal(
                    points=[
                        _point((0.0,) * 6, 1),
                        _point((1.0,) * 6, 1),
                    ]
                )
            )

    def test_rejects_oversized_sequence_before_iterating_points(self) -> None:
        class OversizedPoints:
            def __len__(self):
                return 1001

            def __iter__(self):
                raise AssertionError("oversized points must not be iterated")

        with self.assertRaisesRegex(
            GoalConversionError,
            "1001 points; maximum is 1000",
        ) as raised:
            _convert(_goal(points=OversizedPoints()))

        self.assertIs(
            raised.exception.code,
            TrajectoryErrorCode.INVALID_GOAL,
        )

    def test_rejects_old_header_timestamp(self) -> None:
        with self.assertRaises(GoalConversionError) as raised:
            _convert(_goal(stamp=_duration(90)))
        self.assertIs(
            raised.exception.code, TrajectoryErrorCode.OLD_HEADER_TIMESTAMP
        )

    def test_resolves_default_explicit_and_erased_tolerance(self) -> None:
        tolerances = [
            SimpleNamespace(
                name="j1", position=0.1, velocity=0.0, acceleration=0.0
            ),
            SimpleNamespace(
                name="j2", position=-1.0, velocity=0.0, acceleration=0.0
            ),
        ]
        accepted = _convert(_goal(path_tolerance=tolerances))

        self.assertEqual(accepted.tolerance.path_position_rad[0], 0.1)
        self.assertTrue(math.isinf(accepted.tolerance.path_position_rad[1]))
        self.assertEqual(
            accepted.tolerance.path_position_rad[2:],
            (0.2, 0.2, 0.2, 0.2),
        )

    def test_violated_joint_names_ignores_erased_tolerance(self) -> None:
        violated = violated_joint_names(
            actual_position_rad=(1.0, 1.0),
            desired_position_rad=(0.0, 0.0),
            tolerance_rad=(0.5, math.inf),
            joint_names=("j1", "j2"),
        )
        self.assertEqual(violated, ("j1",))
