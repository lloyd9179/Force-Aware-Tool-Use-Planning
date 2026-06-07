import numpy as np
import pytest

from force_tool_planning.ik import solve_planar_3link_ik
from force_tool_planning.kinematics import ArmModel, forward_kinematics, wrap_to_pi


def assert_pose_equivalent(actual: np.ndarray, expected: np.ndarray) -> None:
    np.testing.assert_allclose(actual[:2], expected[:2], atol=1e-9)
    np.testing.assert_allclose(wrap_to_pi(actual[2] - expected[2]), 0.0, atol=1e-9)


def test_ik_round_trip_for_reachable_poses() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 0.8, 0.4])
    joint_vectors = [
        np.array([0.4, 0.8, -0.3]),
        np.array([-0.7, 1.2, 0.2]),
        np.array([1.0, -0.9, -0.4]),
    ]

    for q_rad in joint_vectors:
        target_pose = forward_kinematics(arm, q_rad)
        candidates = solve_planar_3link_ik(arm, target_pose)

        assert candidates
        for candidate in candidates:
            assert_pose_equivalent(forward_kinematics(arm, candidate), target_pose)


def test_ik_returns_both_elbow_branches_in_deterministic_order() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])
    target_pose = forward_kinematics(arm, [0.3, 1.0, -0.4])

    candidates = solve_planar_3link_ik(arm, target_pose)

    assert len(candidates) == 2
    assert candidates[0][1] > 0.0
    assert candidates[1][1] < 0.0


def test_ik_returns_empty_list_for_unreachable_target() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])

    assert solve_planar_3link_ik(arm, [4.0, 0.0, 0.0]) == []


def test_ik_joint_limit_filtering_discards_outside_branch() -> None:
    unrestricted_arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])
    target_pose = forward_kinematics(unrestricted_arm, [0.3, 1.0, -0.4])
    unrestricted_candidates = solve_planar_3link_ik(unrestricted_arm, target_pose)
    accepted = unrestricted_candidates[0]
    limited_arm = ArmModel(
        link_lengths_m=unrestricted_arm.link_lengths_m,
        joint_limits_rad=np.column_stack((accepted - 0.05, accepted + 0.05)),
    )

    filtered_candidates = solve_planar_3link_ik(
        limited_arm,
        target_pose,
        include_joint_limit_check=True,
    )

    assert len(filtered_candidates) == 1
    np.testing.assert_allclose(filtered_candidates[0], accepted)


def test_ik_returns_one_unique_candidate_at_singular_extension() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])

    candidates = solve_planar_3link_ik(arm, [2.5, 0.0, 0.0])

    assert len(candidates) == 1
    np.testing.assert_allclose(candidates[0], np.zeros(3), atol=1e-12)


def test_ik_validates_arm_and_tolerance() -> None:
    with pytest.raises(ValueError, match="exactly 3 joints"):
        solve_planar_3link_ik(ArmModel(link_lengths_m=[1.0, 1.0]), [1.0, 0.0, 0.0])

    with pytest.raises(ValueError, match="non-negative"):
        solve_planar_3link_ik(
            ArmModel(link_lengths_m=[1.0, 1.0, 0.5]),
            [1.0, 0.0, 0.0],
            atol=-1.0,
        )
