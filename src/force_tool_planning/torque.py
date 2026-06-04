"""Torque feasibility checks using ``tau = J(q).T @ F``."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class TorqueCheckResult:
    """Structured result for per-joint torque limit checks."""

    torque_nm: np.ndarray
    torque_limits_nm: np.ndarray
    margin_nm: np.ndarray
    feasible: bool

    @property
    def violating_joint_indices(self) -> np.ndarray:
        """Indices of joints whose absolute torque exceeds the limit."""

        return np.flatnonzero(self.margin_nm < 0.0)


def joint_torques_from_wrench(jacobian: ArrayLike, wrench: ArrayLike) -> np.ndarray:
    """Compute joint torques from a Jacobian and force/wrench vector.

    Args:
        jacobian: Matrix with shape ``(wrench_dim, n_joints)``.
        wrench: Force or wrench vector with shape ``(wrench_dim,)``.

    Returns:
        Joint torque vector with shape ``(n_joints,)`` in N*m.
    """

    jacobian_array = np.asarray(jacobian, dtype=float)
    wrench_array = np.asarray(wrench, dtype=float)

    if jacobian_array.ndim != 2:
        raise ValueError("jacobian must be a 2D array")
    if wrench_array.ndim != 1:
        raise ValueError("wrench must be a 1D array")
    if jacobian_array.shape[0] != wrench_array.shape[0]:
        raise ValueError(
            "jacobian row count must match wrench dimension: "
            f"{jacobian_array.shape[0]} != {wrench_array.shape[0]}"
        )

    return jacobian_array.T @ wrench_array


def joint_torques_from_force(jacobian_xy: ArrayLike, force_xy_n: ArrayLike) -> np.ndarray:
    """Compute joint torques from a translational Jacobian and ``[Fx, Fy]`` force."""

    return joint_torques_from_wrench(jacobian_xy, force_xy_n)


def check_torque_limits(
    joint_torques_nm: ArrayLike,
    torque_limits_nm: ArrayLike,
    *,
    atol_nm: float = 1e-9,
) -> TorqueCheckResult:
    """Check whether every joint torque is within its absolute torque limit."""

    torque_nm = np.asarray(joint_torques_nm, dtype=float)
    limits_nm = np.asarray(torque_limits_nm, dtype=float)

    if torque_nm.ndim != 1:
        raise ValueError("joint_torques_nm must be a 1D array")
    if limits_nm.shape != torque_nm.shape:
        raise ValueError(f"torque_limits_nm must have shape {torque_nm.shape}")
    if np.any(limits_nm <= 0.0):
        raise ValueError("torque_limits_nm must contain only positive limits")
    if atol_nm < 0.0:
        raise ValueError("atol_nm must be non-negative")

    margin_nm = limits_nm - np.abs(torque_nm)
    feasible = bool(np.all(margin_nm >= -atol_nm))
    return TorqueCheckResult(
        torque_nm=torque_nm,
        torque_limits_nm=limits_nm,
        margin_nm=margin_nm,
        feasible=feasible,
    )


def is_torque_feasible(
    joint_torques_nm: ArrayLike,
    torque_limits_nm: ArrayLike,
    *,
    atol_nm: float = 1e-9,
) -> bool:
    """Return whether every joint torque is within its absolute torque limit."""

    return check_torque_limits(joint_torques_nm, torque_limits_nm, atol_nm=atol_nm).feasible
