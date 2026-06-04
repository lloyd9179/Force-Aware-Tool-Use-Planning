import numpy as np
import pytest

from force_tool_planning.kinematics import (
    ArmModel,
    forward_kinematics,
    joint_positions,
    within_joint_limits,
    wrap_to_pi,
)


def test_joint_positions_for_straight_arm() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])

    positions_m = joint_positions(arm, [0.0, 0.0, 0.0])
    pose = forward_kinematics(arm, [0.0, 0.0, 0.0])

    np.testing.assert_allclose(
        positions_m,
        np.array(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [2.0, 0.0],
                [2.5, 0.0],
            ]
        ),
    )
    np.testing.assert_allclose(pose, np.array([2.5, 0.0, 0.0]))


def test_joint_positions_use_cumulative_joint_angles() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 1.0])
    q_rad = [np.pi / 2.0, -np.pi / 2.0, np.pi / 2.0]

    positions_m = joint_positions(arm, q_rad)
    pose = forward_kinematics(arm, q_rad)

    np.testing.assert_allclose(
        positions_m,
        np.array(
            [
                [0.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
                [1.0, 2.0],
            ]
        ),
        atol=1e-12,
    )
    np.testing.assert_allclose(pose, np.array([1.0, 2.0, np.pi / 2.0]), atol=1e-12)


def test_within_joint_limits_checks_bounds_with_tolerance() -> None:
    arm = ArmModel(
        link_lengths_m=[1.0, 1.0, 1.0],
        joint_limits_rad=[
            [-1.0, 1.0],
            [-0.5, 0.5],
            [-2.0, 2.0],
        ],
    )

    assert within_joint_limits(arm, [1.0, -0.5, 0.0])
    assert not within_joint_limits(arm, [1.1, 0.0, 0.0])
    assert not within_joint_limits(arm, [0.0, -0.6, 0.0])


def test_arm_model_rejects_invalid_shapes() -> None:
    with pytest.raises(ValueError, match="non-empty 1D"):
        ArmModel(link_lengths_m=[])

    with pytest.raises(ValueError, match="positive"):
        ArmModel(link_lengths_m=[1.0, 0.0, 1.0])

    with pytest.raises(ValueError, match="joint_limits_rad"):
        ArmModel(link_lengths_m=[1.0, 1.0], joint_limits_rad=[[-1.0, 1.0]])


def test_wrap_to_pi_wraps_arrays() -> None:
    wrapped = wrap_to_pi(np.array([-3.0 * np.pi, -np.pi / 2.0, 3.0 * np.pi]))

    np.testing.assert_allclose(wrapped, np.array([-np.pi, -np.pi / 2.0, -np.pi]))
