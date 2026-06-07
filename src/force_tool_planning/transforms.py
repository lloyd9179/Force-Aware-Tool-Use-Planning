"""Deterministic planar pose transform utilities.

Planar poses use ``[x_m, y_m, theta_rad]``. Composition follows the frame
convention ``world_T_b = world_T_a compose a_T_b``.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from force_tool_planning.kinematics import wrap_to_pi


def as_pose3(pose: ArrayLike) -> np.ndarray:
    """Validate and return a planar pose as a shape-``(3,)`` float array."""

    pose_array = np.asarray(pose, dtype=float)
    if pose_array.shape != (3,):
        raise ValueError("pose must have shape (3,)")
    if not np.all(np.isfinite(pose_array)):
        raise ValueError("pose must contain only finite values")
    return pose_array


def compose_pose(a: ArrayLike, b: ArrayLike) -> np.ndarray:
    """Return ``a compose b`` for planar transforms ``world_T_a`` and ``a_T_b``."""

    pose_a = as_pose3(a)
    pose_b = as_pose3(b)

    cos_a = np.cos(pose_a[2])
    sin_a = np.sin(pose_a[2])
    return np.array(
        [
            pose_a[0] + cos_a * pose_b[0] - sin_a * pose_b[1],
            pose_a[1] + sin_a * pose_b[0] + cos_a * pose_b[1],
            float(wrap_to_pi(pose_a[2] + pose_b[2])),
        ],
        dtype=float,
    )


def invert_pose(pose: ArrayLike) -> np.ndarray:
    """Return the inverse transform ``b_T_a`` for an input transform ``a_T_b``."""

    pose_array = as_pose3(pose)
    cos_theta = np.cos(pose_array[2])
    sin_theta = np.sin(pose_array[2])
    return np.array(
        [
            -cos_theta * pose_array[0] - sin_theta * pose_array[1],
            sin_theta * pose_array[0] - cos_theta * pose_array[1],
            float(wrap_to_pi(-pose_array[2])),
        ],
        dtype=float,
    )


def relative_pose(a: ArrayLike, b: ArrayLike) -> np.ndarray:
    """Return ``a_T_b = inverse(world_T_a) compose world_T_b``."""

    return compose_pose(invert_pose(a), b)


def transform_path(path: ArrayLike, transform: ArrayLike) -> np.ndarray:
    """Right-compose every ``world_T_a`` path pose with the same ``a_T_b``."""

    path_array = np.asarray(path, dtype=float)
    if path_array.ndim != 2 or path_array.shape[1] != 3:
        raise ValueError("path must have shape (num_waypoints, 3)")
    if not np.all(np.isfinite(path_array)):
        raise ValueError("path must contain only finite values")

    transform_array = as_pose3(transform)
    return np.asarray(
        [compose_pose(pose, transform_array) for pose in path_array],
        dtype=float,
    ).reshape((-1, 3))
