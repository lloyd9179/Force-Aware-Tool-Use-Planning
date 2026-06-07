"""Deterministic analytic inverse kinematics for a planar 3-link arm."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from force_tool_planning.kinematics import ArmModel, within_joint_limits, wrap_to_pi
from force_tool_planning.transforms import as_pose3


def solve_planar_3link_ik(
    arm: ArmModel,
    target_pose: ArrayLike,
    *,
    include_joint_limit_check: bool = False,
    atol: float = 1e-9,
) -> list[np.ndarray]:
    """Return up to two analytic IK branches for a planar 3-link arm.

    The target is ``world_T_ee = [x_m, y_m, theta_rad]``. Candidate order is
    deterministic: positive-``sin(q2)`` elbow branch first, then the negative
    branch. Unreachable targets return an empty list.
    """

    if arm.n_joints != 3:
        raise ValueError("solve_planar_3link_ik requires an arm with exactly 3 joints")
    if atol < 0.0:
        raise ValueError("atol must be non-negative")

    x_m, y_m, theta_rad = as_pose3(target_pose)
    l1_m, l2_m, l3_m = arm.link_lengths_m

    wrist_x_m = x_m - l3_m * np.cos(theta_rad)
    wrist_y_m = y_m - l3_m * np.sin(theta_rad)
    cos_q2 = (
        wrist_x_m**2 + wrist_y_m**2 - l1_m**2 - l2_m**2
    ) / (2.0 * l1_m * l2_m)

    if cos_q2 < -1.0 - atol or cos_q2 > 1.0 + atol:
        return []

    cos_q2 = float(np.clip(cos_q2, -1.0, 1.0))
    sin_q2_magnitude = float(np.sqrt(max(0.0, 1.0 - cos_q2**2)))
    candidates: list[np.ndarray] = []

    for sin_q2 in (sin_q2_magnitude, -sin_q2_magnitude):
        q2_rad = np.arctan2(sin_q2, cos_q2)
        q1_rad = np.arctan2(wrist_y_m, wrist_x_m) - np.arctan2(
            l2_m * sin_q2,
            l1_m + l2_m * cos_q2,
        )
        q3_rad = theta_rad - q1_rad - q2_rad
        candidate = wrap_to_pi([q1_rad, q2_rad, q3_rad])

        if include_joint_limit_check and not within_joint_limits(
            arm,
            candidate,
            atol_rad=atol,
        ):
            continue
        if any(np.allclose(candidate, existing, atol=atol, rtol=0.0) for existing in candidates):
            continue
        candidates.append(candidate)

    return candidates
