import importlib

import numpy as np
import pytest
from myarm_m750_core.adapters.kinematics.geometry_tools import (
    load_urdf_transform_manager,
)
from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.kinematics.math3d import (
    quaternion_wxyz_to_xyzw,
    quaternion_xyzw_to_wxyz,
)


def test_quaternion_boundary_conversion_is_lossless() -> None:
    quaternion_xyzw = np.asarray([0.1, -0.2, 0.3, 0.9])
    np.testing.assert_array_equal(
        quaternion_wxyz_to_xyzw(quaternion_xyzw_to_wxyz(quaternion_xyzw)),
        quaternion_xyzw,
    )


def test_geometry_tools_dependency_is_lazy_and_actionable(
    monkeypatch, repository_root
) -> None:
    real_import_module = importlib.import_module

    def reject_pytransform3d(module_name):
        if module_name.startswith("pytransform3d"):
            raise ImportError("test dependency absence")
        return real_import_module(module_name)

    monkeypatch.setattr(importlib, "import_module", reject_pytransform3d)
    urdf_path = (
        repository_root
        / "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_kinematic.urdf"
    )
    with pytest.raises(KinematicsError, match="geometry-tools"):
        load_urdf_transform_manager(urdf_path)
