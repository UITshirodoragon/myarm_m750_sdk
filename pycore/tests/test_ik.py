import numpy as np


def test_ik_reconstructs_fk_target_from_nearby_seed(kinematics) -> None:
    target_joint_rad = np.array([0.2, -0.3, 0.4, 0.2, -0.2, 0.1])
    target_pose = kinematics.compute_fk(target_joint_rad)
    result = kinematics.solve_ik(target_pose, target_joint_rad + 0.03)
    assert result.succeeded
    reconstructed = kinematics.compute_fk(result.joint_position_rad).as_matrix()
    np.testing.assert_allclose(reconstructed, target_pose.as_matrix(), atol=2.0e-4)
