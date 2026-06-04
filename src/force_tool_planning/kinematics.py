"""Planar arm kinematics for the force-aware tool-use planning demo.

Frame conventions:
- world frame: fixed 2D frame with the robot base at the origin.
- end-effector frame: frame attached to the final link tip.
- tool frame and tool-tip frame are not modeled in this module; later grasp
  transforms should map between these frames and the end-effector frame.

The planar arm state is a vector of revolute joint angles in radians. Link
lengths are expressed in meters. End-effector poses use ``[x_m, y_m, theta_rad]``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class ArmModel:
    """Planar revolute arm model.

    Attributes:
        link_lengths_m: Positive link lengths in meters.
        joint_limits_rad: Optional ``(n_joints, 2)`` lower/upper joint limits in radians.
        torque_limits_nm: Optional positive per-joint torque limits in N*m.
    """

    link_lengths_m: ArrayLike
    joint_limits_rad: ArrayLike | None = None
    torque_limits_nm: ArrayLike | None = None

    def __post_init__(self) -> None:
        link_lengths_m = np.asarray(self.link_lengths_m, dtype=float)
        if link_lengths_m.ndim != 1 or link_lengths_m.size == 0:
            raise ValueError("link_lengths_m must be a non-empty 1D array")
        if np.any(link_lengths_m <= 0.0):
            raise ValueError("link_lengths_m must contain only positive lengths")

        object.__setattr__(self, "link_lengths_m", link_lengths_m)

        if self.joint_limits_rad is not None:
            joint_limits_rad = np.asarray(self.joint_limits_rad, dtype=float)
            expected_shape = (link_lengths_m.size, 2)
            if joint_limits_rad.shape != expected_shape:
                raise ValueError(f"joint_limits_rad must have shape {expected_shape}")
            if np.any(joint_limits_rad[:, 0] > joint_limits_rad[:, 1]):
                raise ValueError("joint limit lower bounds must be <= upper bounds")
            object.__setattr__(self, "joint_limits_rad", joint_limits_rad)

        if self.torque_limits_nm is not None:
            torque_limits_nm = np.asarray(self.torque_limits_nm, dtype=float)
            expected_shape = (link_lengths_m.size,)
            if torque_limits_nm.shape != expected_shape:
                raise ValueError(f"torque_limits_nm must have shape {expected_shape}")
            if np.any(torque_limits_nm <= 0.0):
                raise ValueError("torque_limits_nm must contain only positive limits")
            object.__setattr__(self, "torque_limits_nm", torque_limits_nm)

    @property
    def n_joints(self) -> int:
        """Number of revolute joints in the planar arm."""

        return int(self.link_lengths_m.size)


def as_joint_vector(arm: ArmModel, joint_angles_rad: ArrayLike) -> np.ndarray:
    """Return joint angles as a validated 1D float array."""

    q_rad = np.asarray(joint_angles_rad, dtype=float)
    expected_shape = (arm.n_joints,)
    if q_rad.shape != expected_shape:
        raise ValueError(f"joint_angles_rad must have shape {expected_shape}")
    return q_rad


def wrap_to_pi(angles_rad: ArrayLike) -> np.ndarray:
    """Wrap angles in radians to the interval ``[-pi, pi)``."""

    angles = np.asarray(angles_rad, dtype=float)
    return (angles + np.pi) % (2.0 * np.pi) - np.pi


def joint_positions(arm: ArmModel, joint_angles_rad: ArrayLike) -> np.ndarray:
    """Compute world-frame positions of the base, joints, and end-effector.

    Returns:
        Array with shape ``(n_joints + 1, 2)``. Row 0 is the base at ``[0, 0]``;
        each following row is the next joint or final end-effector position.
    """

    q_rad = as_joint_vector(arm, joint_angles_rad)
    cumulative_angles_rad = np.cumsum(q_rad)
    link_vectors_m = np.column_stack(
        (
            arm.link_lengths_m * np.cos(cumulative_angles_rad),
            arm.link_lengths_m * np.sin(cumulative_angles_rad),
        )
    )

    positions_m = np.zeros((arm.n_joints + 1, 2), dtype=float)
    positions_m[1:] = np.cumsum(link_vectors_m, axis=0)
    return positions_m


def forward_kinematics(arm: ArmModel, joint_angles_rad: ArrayLike) -> np.ndarray:
    """Compute the planar end-effector pose ``[x_m, y_m, theta_rad]``."""

    q_rad = as_joint_vector(arm, joint_angles_rad)
    ee_position_m = joint_positions(arm, q_rad)[-1]
    theta_rad = float(np.sum(q_rad))
    return np.array([ee_position_m[0], ee_position_m[1], theta_rad], dtype=float)


def within_joint_limits(
    arm: ArmModel,
    joint_angles_rad: ArrayLike,
    *,
    atol_rad: float = 1e-9,
) -> bool:
    """Return whether joint angles are inside the model's joint limits.

    If the arm has no joint limits, every validated joint vector is accepted.
    """

    q_rad = as_joint_vector(arm, joint_angles_rad)
    if arm.joint_limits_rad is None:
        return True

    lower_rad = arm.joint_limits_rad[:, 0] - atol_rad
    upper_rad = arm.joint_limits_rad[:, 1] + atol_rad
    return bool(np.all((q_rad >= lower_rad) & (q_rad <= upper_rad)))
