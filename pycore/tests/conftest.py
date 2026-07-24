from pathlib import Path

import pytest

from myarm_m750_core.runtime.config import load_sdk_config
from myarm_m750_core.domain.kinematics import PoeKinematics


@pytest.fixture(scope="session")
def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def sdk_config(repository_root: Path):
    return load_sdk_config(str(repository_root / "pycore" / "config" / "default.yaml"))


@pytest.fixture(scope="session")
def kinematics(sdk_config):
    return PoeKinematics.from_urdf(
        urdf_path=sdk_config.robot.urdf_path,
        base_link=sdk_config.robot.base_link,
        end_link=sdk_config.robot.end_link,
        joint_names=sdk_config.robot.joint_names,
    )
