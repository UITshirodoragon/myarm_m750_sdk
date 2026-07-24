import numpy as np


def test_zero_pose_matches_supplied_urdf_home_transform(kinematics) -> None:
    pose = kinematics.compute_fk([0.0] * 6)
    np.testing.assert_allclose(pose.translation_m, [0.58191, 0.0, 0.4769], atol=1.0e-9)
    np.testing.assert_allclose(pose.as_matrix(), kinematics.home_transform, atol=1.0e-10)


def test_jacobian_has_expected_shape_and_finite_values(kinematics) -> None:
    jacobian = kinematics.compute_jacobian([0.2, -0.3, 0.4, 0.2, -0.2, 0.1])
    assert jacobian.shape == (6, 6)
    assert np.all(np.isfinite(jacobian))


def test_jacobian_linear_part_matches_finite_difference(kinematics) -> None:
    joint_position_rad = np.array([0.2, -0.3, 0.4, 0.2, -0.2, 0.1])
    jacobian = kinematics.compute_jacobian(joint_position_rad)
    epsilon_rad = 1.0e-7
    for joint_index in range(6):
        positive = joint_position_rad.copy()
        negative = joint_position_rad.copy()
        positive[joint_index] += epsilon_rad
        negative[joint_index] -= epsilon_rad
        positive_position = np.asarray(kinematics.compute_fk(positive).translation_m)
        negative_position = np.asarray(kinematics.compute_fk(negative).translation_m)
        finite_difference = (positive_position - negative_position) / (2.0 * epsilon_rad)
        np.testing.assert_allclose(
            jacobian[3:, joint_index], finite_difference, atol=2.0e-6
        )
