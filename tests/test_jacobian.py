import numpy as np
import pytest

from force_tool_planning.jacobian import planar_jacobian, translational_jacobian_xy
from force_tool_planning.kinematics import ArmModel, forward_kinematics


def test_translational_jacobian_for_straight_three_link_arm() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 1.0])

    jacobian_xy = translational_jacobian_xy(arm, [0.0, 0.0, 0.0])
    jacobian_planar = planar_jacobian(arm, [0.0, 0.0, 0.0])

    np.testing.assert_allclose(
        jacobian_xy,
        np.array(
            [
                [0.0, 0.0, 0.0],
                [3.0, 2.0, 1.0],
            ]
        ),
        atol=1e-12,
    )
    np.testing.assert_allclose(jacobian_planar[2], np.ones(3))


def test_planar_jacobian_matches_finite_difference() -> None:
    arm = ArmModel(link_lengths_m=[0.7, 0.5, 0.3])
    q_rad = np.array([0.4, -0.9, 1.2])
    epsilon = 1e-7

    numeric = np.zeros((3, arm.n_joints), dtype=float)
    for joint_index in range(arm.n_joints):
        delta = np.zeros(arm.n_joints, dtype=float)
        delta[joint_index] = epsilon
        pose_plus = forward_kinematics(arm, q_rad + delta)
        pose_minus = forward_kinematics(arm, q_rad - delta)
        numeric[:, joint_index] = (pose_plus - pose_minus) / (2.0 * epsilon)

    np.testing.assert_allclose(planar_jacobian(arm, q_rad), numeric, atol=1e-8)


def test_jacobian_rejects_wrong_joint_vector_shape() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 1.0])

    with pytest.raises(ValueError, match="joint_angles_rad"):
        planar_jacobian(arm, [0.0, 0.0])
