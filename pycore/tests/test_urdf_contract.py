import xml.etree.ElementTree as ET


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)
EXPECTED_AXES = (
    "0 0 1",
    "0 1 0",
    "0 1 0",
    "1 0 0",
    "0 1 0",
    "1 0 0",
)


def _vector(text):
    return tuple(float(value) for value in text.split())


def _joint_contract(path):
    root = ET.parse(str(path)).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    result = {}
    for name in ARM_JOINTS:
        joint = joints[name]
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        result[name] = (
            _vector(origin.attrib["xyz"]),
            _vector(origin.attrib["rpy"]),
            _vector(axis.attrib["xyz"]),
            (float(limit.attrib["lower"]), float(limit.attrib["upper"])),
        )
    return result


def test_supplied_and_standalone_urdf_share_kinematic_contract(repository_root) -> None:
    urdf_dir = repository_root / "ros2/src/myarm_m750_description/urdf"
    supplied = _joint_contract(urdf_dir / "myarm_m750_poe_v3_2.urdf")
    standalone = _joint_contract(urdf_dir / "myarm_m750_standalone.urdf")
    assert supplied == standalone


def test_arm_origins_and_axes_are_canonical_poe(repository_root) -> None:
    path = repository_root / "ros2/src/myarm_m750_description/urdf/myarm_m750_poe_v3_2.urdf"
    contract = _joint_contract(path)
    for name, expected_axis in zip(ARM_JOINTS, EXPECTED_AXES):
        assert contract[name][1] == (0.0, 0.0, 0.0)
        assert contract[name][2] == _vector(expected_axis)
