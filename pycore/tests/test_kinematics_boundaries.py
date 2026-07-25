import math
from pathlib import Path
from types import SimpleNamespace
from typing import List, Sequence

import myarm_m750_core.adapters.kinematics.geometry_tools as geometry_tools
import myarm_m750_core.adapters.kinematics.pinocchio_provider as pinocchio_provider
import numpy as np
import pytest
from myarm_m750_core.adapters.kinematics import (
    PinocchioKinematics,
    create_kinematics_provider,
)
from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.kinematics import PoeKinematics
from myarm_m750_core.domain.kinematics.math3d import (
    matrix_to_quaternion_xyzw,
    quaternion_wxyz_to_xyzw,
    quaternion_xyzw_to_matrix,
    quaternion_xyzw_to_wxyz,
    rotation_exp,
    rotation_log_vector,
    rotation_x,
    rotation_y,
    rotation_z,
    twist_exp,
)
from myarm_m750_core.domain.kinematics.model import (
    fingerprint_urdf_path,
    normalized_kinematic_contract,
)
from myarm_m750_core.domain.kinematics.solver import (
    DampedLeastSquaresSettings,
    solve_damped_least_squares,
)
from myarm_m750_core.domain.models import JointLimits, RigidTransform

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


def _canonical_urdf(repository_root: Path) -> Path:
    return (
        repository_root
        / "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_kinematic.urdf"
    )


def _write_serial_chain(
    path: Path,
    joint_types: Sequence[str],
    axes: Sequence[str],
    include_limits: bool = True,
) -> Path:
    links = [f'  <link name="link{index}"/>' for index in range(7)]
    joints: List[str] = []
    for index, (joint_type, axis) in enumerate(zip(joint_types, axes), start=1):
        limit = (
            '    <limit lower="-1" upper="1" velocity="1" effort="1"/>\n'
            if include_limits and joint_type in ("revolute", "prismatic")
            else ""
        )
        joints.append(
            f'  <joint name="joint{index}" type="{joint_type}">\n'
            f'    <parent link="link{index - 1}"/>\n'
            f'    <child link="link{index}"/>\n'
            '    <origin xyz="0.1 0 0" rpy="0 0 0"/>\n'
            f'    <axis xyz="{axis}"/>\n'
            f"{limit}"
            "  </joint>"
        )
    links_xml = "\n".join(links)
    joints_xml = "\n".join(joints)
    path.write_text(
        f'<robot name="test">\n{links_xml}\n{joints_xml}\n</robot>\n',
        encoding="utf-8",
    )
    return path


class _FakeTransformManager:
    def __init__(self) -> None:
        self.load_arguments = None
        self.joint_positions = {}

    def load_urdf(self, urdf_xml, mesh_path, package_dir) -> None:
        self.load_arguments = (urdf_xml, mesh_path, package_dir)

    def set_joint(self, joint_name, joint_position_rad) -> None:
        self.joint_positions[joint_name] = joint_position_rad


def _install_fake_pytransform3d(monkeypatch, version: str = "3.16.0") -> None:
    package = SimpleNamespace(__version__=version)
    urdf_module = SimpleNamespace(UrdfTransformManager=_FakeTransformManager)

    def fake_import(module_name):
        if module_name == "pytransform3d":
            return package
        if module_name == "pytransform3d.urdf":
            return urdf_module
        raise ImportError(module_name)

    monkeypatch.setattr(geometry_tools.importlib, "import_module", fake_import)


def test_geometry_tools_loads_plain_and_package_uris(
    monkeypatch, tmp_path, repository_root
) -> None:
    _install_fake_pytransform3d(monkeypatch)
    plain_urdf = tmp_path / "plain.urdf"
    plain_urdf.write_text(
        '<robot name="plain"><link name="base_link"/></robot>',
        encoding="utf-8",
    )
    plain_manager = geometry_tools.load_urdf_transform_manager(plain_urdf, {"joint1": 0.25})
    assert plain_manager.load_arguments[1] == str(tmp_path)
    assert plain_manager.load_arguments[2] is None
    assert plain_manager.joint_positions == {"joint1": 0.25}

    full_urdf = (
        repository_root
        / "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_full.urdf"
    )
    package_manager = geometry_tools.load_urdf_transform_manager(full_urdf)
    assert package_manager.load_arguments[1] is None
    assert package_manager.load_arguments[2] == str(repository_root / "ros2/src") + "/"


def test_geometry_tools_rejects_version_and_missing_model(monkeypatch, tmp_path) -> None:
    _install_fake_pytransform3d(monkeypatch, version="3.15.0")
    with pytest.raises(KinematicsError, match="Unsupported pytransform3d version"):
        geometry_tools.load_urdf_transform_manager(tmp_path / "missing.urdf")

    _install_fake_pytransform3d(monkeypatch)
    with pytest.raises(KinematicsError, match="Could not read URDF"):
        geometry_tools.load_urdf_transform_manager(tmp_path / "missing.urdf")


def test_pinocchio_loader_rejects_absence_and_wrong_version(monkeypatch) -> None:
    def missing_pinocchio(_module_name):
        raise OSError("ABI unavailable")

    monkeypatch.setattr(pinocchio_provider.importlib, "import_module", missing_pinocchio)
    with pytest.raises(pinocchio_provider.PinocchioUnavailableError, match="2.6.17"):
        pinocchio_provider._load_pinocchio()

    monkeypatch.setattr(
        pinocchio_provider.importlib,
        "import_module",
        lambda _module_name: SimpleNamespace(__version__="3.0.0"),
    )
    with pytest.raises(KinematicsError, match="Unsupported Pinocchio version"):
        pinocchio_provider._load_pinocchio()


def test_pinocchio_model_and_input_boundaries(repository_root, tmp_path) -> None:
    pytest.importorskip("pinocchio")
    urdf_path = _canonical_urdf(repository_root)
    with pytest.raises(KinematicsError, match="six unique joints"):
        PinocchioKinematics(
            urdf_path,
            "base_link",
            "tool0",
            ARM_JOINTS[:-1] + (ARM_JOINTS[0],),
        )
    with pytest.raises(KinematicsError, match="does not contain frame"):
        PinocchioKinematics(urdf_path, "base_link", "missing_tool", ARM_JOINTS)
    with pytest.raises(KinematicsError, match="does not contain joint"):
        PinocchioKinematics(
            urdf_path,
            "base_link",
            "tool0",
            ARM_JOINTS[:-1] + ("missing_joint",),
        )
    with pytest.raises(KinematicsError, match="model order"):
        PinocchioKinematics(
            urdf_path,
            "base_link",
            "tool0",
            tuple(reversed(ARM_JOINTS)),
        )
    malformed_urdf = tmp_path / "malformed.urdf"
    malformed_urdf.write_text("<robot", encoding="utf-8")
    with pytest.raises(KinematicsError, match="could not build URDF"):
        PinocchioKinematics(malformed_urdf, "base_link", "tool0", ARM_JOINTS)

    provider = PinocchioKinematics(urdf_path, "base_link", "tool0", ARM_JOINTS)
    with pytest.raises(ValueError, match="six values"):
        provider.compute_fk([0.0] * 5)
    with pytest.raises(ValueError, match="finite"):
        provider.compute_jacobian([0.0, 0.0, math.nan, 0.0, 0.0, 0.0])
    assert provider.compute_singularity_score([0.0] * 6) >= 0.0


def test_factory_can_explicitly_select_poe(repository_root) -> None:
    provider = create_kinematics_provider(
        urdf_path=_canonical_urdf(repository_root),
        base_link="base_link",
        end_link="tool0",
        joint_names=ARM_JOINTS,
        prefer_pinocchio=False,
    )
    assert isinstance(provider, PoeKinematics)


def test_poe_supports_prismatic_and_continuous_chain(tmp_path) -> None:
    joint_types = (
        "continuous",
        "revolute",
        "revolute",
        "revolute",
        "revolute",
        "prismatic",
    )
    axes = ("0 0 1", "0 1 0", "0 1 0", "1 0 0", "0 1 0", "1 0 0")
    urdf_path = _write_serial_chain(tmp_path / "mixed.urdf", joint_types, axes)
    joint_names = tuple(f"joint{index}" for index in range(1, 7))
    provider = PoeKinematics.from_urdf(urdf_path, "link0", "link6", joint_names)
    assert provider.screw_axes_space.shape == (6, 6)
    assert np.allclose(provider.screw_axes_space[:3, -1], 0.0)
    assert provider.joint_limits.lower_rad[0] == -math.pi
    assert provider.compute_singularity_score([0.0] * 6) >= 0.0
    home_transform = provider.home_transform
    home_transform[0, 0] = 99.0
    assert provider.home_transform[0, 0] != 99.0
    screw_axes = provider.screw_axes_space
    screw_axes[0, 0] = 99.0
    assert provider.screw_axes_space[0, 0] != 99.0


@pytest.mark.parametrize(
    "joint_types,axes,include_limits,expected_message",
    [
        (
            ("revolute",) * 6,
            ("0 0 0",) + ("0 0 1",) * 5,
            True,
            "zero rotation axis",
        ),
        (
            ("prismatic",) + ("revolute",) * 5,
            ("0 0 0",) + ("0 0 1",) * 5,
            True,
            "zero translation axis",
        ),
        (
            ("planar",) + ("revolute",) * 5,
            ("0 0 1",) * 6,
            True,
            "Unsupported URDF joint type",
        ),
        (
            ("revolute",) * 6,
            ("0 0 1",) * 6,
            False,
            "missing a limit element",
        ),
    ],
)
def test_poe_rejects_invalid_joint_contracts(
    tmp_path, joint_types, axes, include_limits, expected_message
) -> None:
    urdf_path = _write_serial_chain(
        tmp_path / (expected_message.replace(" ", "_") + ".urdf"),
        joint_types,
        axes,
        include_limits,
    )
    joint_names = tuple(f"joint{index}" for index in range(1, 7))
    with pytest.raises(KinematicsError, match=expected_message):
        PoeKinematics.from_urdf(urdf_path, "link0", "link6", joint_names)


def test_poe_rejects_malformed_graph_and_dimensions(tmp_path) -> None:
    missing_path = tmp_path / "missing.urdf"
    with pytest.raises(KinematicsError, match="Could not parse URDF"):
        PoeKinematics.from_urdf(missing_path, "link0", "link6", ("joint1",) * 6)

    invalid_vector = tmp_path / "invalid_vector.urdf"
    invalid_vector.write_text(
        '<robot name="bad"><link name="a"/><link name="b"/>'
        '<joint name="j" type="fixed"><parent link="a"/>'
        '<child link="b"/><origin xyz="0 0"/></joint></robot>',
        encoding="utf-8",
    )
    with pytest.raises(KinematicsError, match="exactly three"):
        PoeKinematics.from_urdf(invalid_vector, "a", "b", ("j",) * 6)

    missing_parent = tmp_path / "missing_parent.urdf"
    missing_parent.write_text(
        '<robot name="bad"><link name="a"/><joint name="j" type="fixed">'
        '<child link="a"/></joint></robot>',
        encoding="utf-8",
    )
    with pytest.raises(KinematicsError, match="name, parent, and child"):
        PoeKinematics.from_urdf(missing_parent, "base", "a", ("j",) * 6)

    cycle = tmp_path / "cycle.urdf"
    cycle.write_text(
        '<robot name="cycle"><link name="a"/><link name="b"/>'
        '<joint name="a_to_b" type="fixed"><parent link="a"/>'
        '<child link="b"/></joint><joint name="b_to_a" type="fixed">'
        '<parent link="b"/><child link="a"/></joint></robot>',
        encoding="utf-8",
    )
    with pytest.raises(KinematicsError, match="Cycle detected"):
        PoeKinematics.from_urdf(cycle, "unreachable", "a", ("joint",) * 6)

    limits = JointLimits(lower_rad=(-1.0,) * 6, upper_rad=(1.0,) * 6)
    with pytest.raises(KinematicsError, match="six 6D screw axes"):
        PoeKinematics(
            ARM_JOINTS,
            np.zeros((5, 6)),
            np.eye(4),
            limits,
            "base_link",
            "tool0",
        )
    with pytest.raises(KinematicsError, match="shape"):
        PoeKinematics(
            ARM_JOINTS,
            np.zeros((6, 6)),
            np.eye(3),
            limits,
            "base_link",
            "tool0",
        )


def test_poe_rejects_wrong_chain_and_joint_vectors(tmp_path, repository_root) -> None:
    provider = PoeKinematics.from_urdf(
        _canonical_urdf(repository_root), "base_link", "tool0", ARM_JOINTS
    )
    with pytest.raises(ValueError, match="six values"):
        provider.compute_fk([0.0] * 5)
    with pytest.raises(ValueError, match="finite"):
        provider.compute_jacobian([0.0, 0.0, 0.0, math.inf, 0.0, 0.0])

    five_joint_path = _write_serial_chain(
        tmp_path / "five.urdf",
        ("revolute",) * 5,
        ("0 0 1",) * 5,
    )
    five_names = tuple(f"joint{index}" for index in range(1, 6))
    with pytest.raises(KinematicsError, match="exactly six actuated joints"):
        PoeKinematics.from_urdf(five_joint_path, "link0", "link5", five_names)
    with pytest.raises(KinematicsError, match="does not match config"):
        PoeKinematics.from_urdf(
            _canonical_urdf(repository_root),
            "base_link",
            "tool0",
            tuple(reversed(ARM_JOINTS)),
        )
    with pytest.raises(KinematicsError, match="No URDF chain"):
        PoeKinematics.from_urdf(
            _canonical_urdf(repository_root),
            "base_link",
            "missing_link",
            ARM_JOINTS,
        )


@pytest.mark.parametrize(
    "settings_kwargs,expected_message",
    [
        ({"max_iterations": 0}, "max_iterations"),
        ({"damping": 0.0}, "damping"),
        ({"max_step_rad": 0.0}, "max_step_rad"),
        ({"position_tolerance_m": 0.0}, "position_tolerance_m"),
        ({"orientation_tolerance_rad": 0.0}, "orientation_tolerance_rad"),
    ],
)
def test_dls_settings_reject_non_positive_values(settings_kwargs, expected_message) -> None:
    with pytest.raises(ValueError, match=expected_message):
        DampedLeastSquaresSettings(**settings_kwargs)


def test_dls_input_failure_and_pseudoinverse_fallback(monkeypatch) -> None:
    limits = JointLimits(lower_rad=(-1.0,) * 6, upper_rad=(1.0,) * 6)
    target = RigidTransform(
        translation_m=(0.5, 0.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    settings = DampedLeastSquaresSettings(max_iterations=1, max_step_rad=0.01)

    def identity_fk(_joint_position) -> np.ndarray:
        return np.eye(4)

    def identity_jacobian(_joint_position) -> np.ndarray:
        return np.eye(6)

    with pytest.raises(ValueError, match="wrong number"):
        solve_damped_least_squares(
            target,
            [0.0] * 5,
            limits,
            identity_fk,
            identity_jacobian,
            settings,
            expected_parent_frame="base_link",
            expected_child_frame="tool0",
        )
    with pytest.raises(ValueError, match="finite"):
        solve_damped_least_squares(
            target,
            [0.0, 0.0, 0.0, 0.0, 0.0, math.nan],
            limits,
            identity_fk,
            identity_jacobian,
            settings,
            expected_parent_frame="base_link",
            expected_child_frame="tool0",
        )

    wrong_frame_target = RigidTransform(
        translation_m=(0.5, 0.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        parent_frame="map",
        child_frame="tool0",
    )
    with pytest.raises(KinematicsError, match=r"base_link->tool0.*map->tool0"):
        solve_damped_least_squares(
            wrong_frame_target,
            [0.0] * 6,
            limits,
            identity_fk,
            identity_jacobian,
            settings,
            expected_parent_frame="base_link",
            expected_child_frame="tool0",
        )

    def singular_solve(_matrix, _vector):
        raise np.linalg.LinAlgError("forced singular solve")

    monkeypatch.setattr(np.linalg, "solve", singular_solve)
    result = solve_damped_least_squares(
        target,
        [0.0] * 6,
        limits,
        identity_fk,
        identity_jacobian,
        settings,
        expected_parent_frame="base_link",
        expected_child_frame="tool0",
    )
    assert not result.succeeded
    assert result.iterations == 1
    assert "did not converge" in result.message


def test_model_normalizer_rejects_invalid_xml_and_graph(tmp_path) -> None:
    with pytest.raises(KinematicsError, match="Could not parse URDF XML"):
        normalized_kinematic_contract("<robot")
    with pytest.raises(KinematicsError, match="root element"):
        normalized_kinematic_contract("<not_robot/>")
    with pytest.raises(KinematicsError, match="parent and child"):
        normalized_kinematic_contract(
            '<robot name="bad"><link name="base"/><joint name="bad" type="fixed"/></robot>'
        )
    with pytest.raises(KinematicsError, match="Could not read URDF"):
        fingerprint_urdf_path(tmp_path / "missing.urdf")


def test_math3d_boundary_conventions() -> None:
    np.testing.assert_array_equal(rotation_exp((0.0, 0.0, 0.0), 1.0), np.eye(3))
    with pytest.raises(ValueError, match="six values"):
        twist_exp((0.0,) * 5, 1.0)
    translation = twist_exp((0.0, 0.0, 0.0, 1.0, 0.0, 0.0), 0.25)
    np.testing.assert_allclose(translation[:3, 3], (0.25, 0.0, 0.0))
    with pytest.raises(ValueError, match="norm"):
        quaternion_xyzw_to_matrix((0.0, 0.0, 0.0, 0.0))

    for rotation in (rotation_x(math.pi), rotation_y(math.pi), rotation_z(math.pi)):
        quaternion = matrix_to_quaternion_xyzw(rotation)
        np.testing.assert_allclose(
            quaternion_xyzw_to_matrix(quaternion), rotation, atol=1.0e-12
        )
    np.testing.assert_allclose(
        np.linalg.norm(rotation_log_vector(rotation_x(math.pi))), math.pi
    )
    np.testing.assert_allclose(
        rotation_log_vector(rotation_z(0.25)), (0.0, 0.0, 0.25), atol=1.0e-12
    )
    assert np.linalg.norm(rotation_log_vector(-np.eye(3))) == pytest.approx(math.pi)

    for converter in (quaternion_xyzw_to_wxyz, quaternion_wxyz_to_xyzw):
        with pytest.raises(ValueError, match="four values"):
            converter((0.0, 0.0, 1.0))
        with pytest.raises(ValueError, match="finite"):
            converter((0.0, 0.0, math.nan, 1.0))
