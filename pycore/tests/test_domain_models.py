import math

import numpy as np
import pytest

from myarm_m750_core.domain.models import JointState, RigidTransform


def test_rigid_transform_normalizes_quaternion_and_round_trips_matrix() -> None:
    transform = RigidTransform(
        translation_m=(0.1, -0.2, 0.3),
        quaternion_xyzw=(0.0, 0.0, 0.0, 2.0),
    )
    assert transform.quaternion_xyzw == (0.0, 0.0, 0.0, 1.0)
    reconstructed = RigidTransform.from_matrix(transform.as_matrix())
    np.testing.assert_allclose(reconstructed.as_matrix(), transform.as_matrix())


def test_domain_models_reject_wrong_vector_sizes() -> None:
    with pytest.raises(ValueError):
        JointState(position_rad=(0.0,) * 5)
    with pytest.raises(ValueError):
        RigidTransform(translation_m=(0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))


def test_joint_state_age_is_never_negative() -> None:
    state = JointState(position_rad=(0.0,) * 6, timestamp_s=10.0)
    assert math.isclose(state.age_s(now_s=9.0), 0.0)
