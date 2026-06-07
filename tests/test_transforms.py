import numpy as np
import pytest

from force_tool_planning.transforms import (
    as_pose3,
    compose_pose,
    invert_pose,
    relative_pose,
    transform_path,
)


def test_identity_composition_returns_original_pose() -> None:
    pose = np.array([0.7, -0.2, 0.6])

    np.testing.assert_allclose(compose_pose([0.0, 0.0, 0.0], pose), pose)
    np.testing.assert_allclose(compose_pose(pose, [0.0, 0.0, 0.0]), pose)


def test_pose_composition_uses_first_pose_frame() -> None:
    world_T_a = np.array([1.0, 2.0, np.pi / 2.0])
    a_T_b = np.array([0.5, 0.0, np.pi / 2.0])

    world_T_b = compose_pose(world_T_a, a_T_b)

    np.testing.assert_allclose(world_T_b, np.array([1.0, 2.5, -np.pi]), atol=1e-12)


def test_pose_inverse_composes_to_identity() -> None:
    pose = np.array([0.8, -0.5, 1.1])

    np.testing.assert_allclose(compose_pose(pose, invert_pose(pose)), np.zeros(3), atol=1e-12)


def test_relative_pose_recovers_composed_transform() -> None:
    world_T_a = np.array([0.3, 0.9, -0.7])
    a_T_b = np.array([-0.4, 0.2, 1.0])

    np.testing.assert_allclose(
        relative_pose(world_T_a, compose_pose(world_T_a, a_T_b)),
        a_T_b,
        atol=1e-12,
    )


def test_transform_path_preserves_waypoint_count() -> None:
    path = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, np.pi / 2.0],
            [1.0, 1.0, np.pi],
        ]
    )

    transformed = transform_path(path, [0.2, 0.0, 0.0])

    assert transformed.shape == path.shape
    np.testing.assert_allclose(transformed[0], np.array([0.2, 0.0, 0.0]))
    np.testing.assert_allclose(transformed[1], np.array([1.0, 0.2, np.pi / 2.0]))


def test_pose_and_path_validation_reject_invalid_inputs() -> None:
    with pytest.raises(ValueError, match=r"shape \(3,\)"):
        as_pose3([1.0, 2.0])

    with pytest.raises(ValueError, match="finite"):
        as_pose3([1.0, np.nan, 0.0])

    with pytest.raises(ValueError, match="num_waypoints"):
        transform_path(np.zeros((2, 2)), [0.0, 0.0, 0.0])
