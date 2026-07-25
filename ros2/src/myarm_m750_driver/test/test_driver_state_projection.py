"""Canonical ROS joint-state projection regression tests."""

import unittest

from myarm_m750_driver.driver_node import _complete_model_joint_state


class DriverStateProjectionTest(unittest.TestCase):
    """Verify the passive gripper state has one authoritative publisher."""

    def test_appends_independent_neutral_gripper_joint(self) -> None:
        names, positions = _complete_model_joint_state(
            ("joint1", "joint2"),
            (0.1, -0.2),
        )

        self.assertEqual(
            names,
            ["joint1", "joint2", "left_gripper_joint"],
        )
        self.assertEqual(positions, [0.1, -0.2, 0.0])

    def test_does_not_duplicate_gripper_joint(self) -> None:
        names, positions = _complete_model_joint_state(
            ("joint1", "left_gripper_joint"),
            (0.1, 0.02),
        )

        self.assertEqual(names, ["joint1", "left_gripper_joint"])
        self.assertEqual(positions, [0.1, 0.02])

    def test_rejects_dimension_mismatch(self) -> None:
        with self.assertRaisesRegex(ValueError, "dimensions differ"):
            _complete_model_joint_state(("joint1",), ())


if __name__ == "__main__":
    unittest.main()
