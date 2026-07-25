"""MoveIt Foxy planning, collision, and controller source contracts."""

import unittest
import xml.etree.ElementTree as element_tree
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


class MoveItConfigContractTest(unittest.TestCase):
    """Verify the hand-written Foxy config has one guarded execution path."""

    def setUp(self) -> None:
        self.moveit_share = Path(
            get_package_share_directory("myarm_m750_moveit_config")
        )
        self.description_share = Path(
            get_package_share_directory("myarm_m750_description")
        )

    def test_srdf_chain_kinematics_and_controller_route(self) -> None:
        srdf = element_tree.parse(
            self.moveit_share / "config" / "myarm_m750.srdf"
        ).getroot()
        chain = srdf.find("./group[@name='arm']/chain")
        self.assertIsNotNone(chain)
        self.assertEqual(chain.attrib["base_link"], "base_link")
        self.assertEqual(chain.attrib["tip_link"], "tool0")
        disabled_pairs = {
            (entry.attrib["link1"], entry.attrib["link2"])
            for entry in srdf.findall("./disable_collisions")
        }
        self.assertIn(
            ("wrist_link", "gripper_base_link"),
            disabled_pairs,
        )
        self.assertIn(
            ("left_gripper_link", "right_gripper_link"),
            disabled_pairs,
        )

        kinematics = yaml.safe_load(
            (self.moveit_share / "config" / "kinematics.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            kinematics["arm"]["kinematics_solver"],
            "kdl_kinematics_plugin/KDLKinematicsPlugin",
        )

        controllers = yaml.safe_load(
            (
                self.moveit_share / "config" / "moveit_controllers.yaml"
            ).read_text(encoding="utf-8")
        )
        controller = controllers["moveit_simple_controller_manager"][
            "myarm_m750"
        ]
        self.assertEqual(controller["type"], "FollowJointTrajectory")
        self.assertEqual(controller["action_ns"], "follow_joint_trajectory")
        self.assertEqual(tuple(controller["joints"]), _ARM_JOINTS)
        self.assertEqual(
            "/{0}/{1}".format(
                controllers["moveit_simple_controller_manager"][
                    "controller_names"
                ][0],
                controller["action_ns"],
            ),
            "/myarm_m750/follow_joint_trajectory",
        )

    def test_collision_geometry_is_primitive_and_not_detailed_mesh(self) -> None:
        urdf = element_tree.parse(
            self.description_share
            / "urdf"
            / "generated"
            / "myarm_m750_lightweight.urdf"
        ).getroot()
        geometries = urdf.findall("./link/collision/geometry")

        self.assertGreaterEqual(len(geometries), 9)
        self.assertFalse(
            any(geometry.find("mesh") is not None for geometry in geometries)
        )
        self.assertTrue(
            all(
                any(
                    geometry.find(shape) is not None
                    for shape in ("box", "cylinder", "sphere")
                )
                for geometry in geometries
            )
        )

    def test_launches_separate_plan_only_and_guarded_mock_execution(self) -> None:
        plan_source = (
            self.moveit_share / "launch" / "plan_only.launch.py"
        ).read_text(encoding="utf-8")
        mock_source = (
            self.moveit_share / "launch" / "mock_execution.launch.py"
        ).read_text(encoding="utf-8")
        composition_source = (
            self.moveit_share
            / "lib"
            / "python3.8"
            / "site-packages"
            / "myarm_m750_moveit_config"
            / "plan_only.py"
        )
        if not composition_source.is_file():
            composition_source = (
                Path(__file__).resolve().parents[1]
                / "myarm_m750_moveit_config"
                / "plan_only.py"
            )
        composition = composition_source.read_text(encoding="utf-8")

        self.assertIn("allow_trajectory_execution=False", plan_source)
        self.assertNotIn("myarm_m750_driver", plan_source)
        self.assertIn("allow_trajectory_execution=True", mock_source)
        self.assertIn("enable_command_interfaces", mock_source)
        self.assertIn("use_real_hardware", mock_source)
        self.assertIn("load_model_catalog", composition)
        self.assertNotIn("MoveItConfigsBuilder", composition)
        self.assertIn('{"move_group": ompl_pipeline}', composition)
        self.assertNotIn('"planning_pipelines"', composition)


if __name__ == "__main__":
    unittest.main()
