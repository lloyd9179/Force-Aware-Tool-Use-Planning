"""Planar geometric Jacobians for force-aware tool-use planning."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from force_tool_planning.kinematics import ArmModel, as_joint_vector


def planar_jacobian(arm: ArmModel, joint_angles_rad: ArrayLike) -> np.ndarray:
    """Compute the planar end-effector Jacobian.

    The returned matrix has shape ``(3, n_joints)`` and maps joint velocity to
    end-effector twist ``[x_dot_m_s, y_dot_m_s, theta_dot_rad_s]`` in the world
    frame. It can also be used with a planar wrench ``[Fx_N, Fy_N, Mz_Nm]``:

    ``tau_nm = J(q).T @ wrench``
    """

    q_rad = as_joint_vector(arm, joint_angles_rad)
    cumulative_angles_rad = np.cumsum(q_rad)

    jacobian = np.zeros((3, arm.n_joints), dtype=float)
    sin_terms = np.sin(cumulative_angles_rad)
    cos_terms = np.cos(cumulative_angles_rad)

    for joint_index in range(arm.n_joints):
        affected_lengths_m = arm.link_lengths_m[joint_index:]
        jacobian[0, joint_index] = -float(np.dot(affected_lengths_m, sin_terms[joint_index:]))
        jacobian[1, joint_index] = float(np.dot(affected_lengths_m, cos_terms[joint_index:]))
        jacobian[2, joint_index] = 1.0

    return jacobian


def translational_jacobian_xy(arm: ArmModel, joint_angles_rad: ArrayLike) -> np.ndarray:
    """Compute the ``2 x n`` translational Jacobian for ``[x, y]`` velocity."""

    return planar_jacobian(arm, joint_angles_rad)[:2]
