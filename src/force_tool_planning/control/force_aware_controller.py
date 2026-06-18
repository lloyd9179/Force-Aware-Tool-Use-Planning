"""Force-aware tool-tip controller for Phase 3 contact execution."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


_ORTHOGONALITY_TOL = 1e-6


@dataclass(frozen=True)
class ForceAwareController:
    """Track tangential motion and correct normal motion from force error.

    The command is a 2D tool-tip velocity. Tangential motion is computed from
    desired position and velocity projected onto the surface tangent. Normal
    motion uses this sign convention:

    - the surface normal points away from the surface into free space;
    - ``force_error = desired_normal_force - measured_normal_force``;
    - positive force error means measured force is too low;
    - positive normal correction commands motion opposite the normal, into
      contact.
    """

    kp_tangent: float
    kd_tangent: float
    force_gain_mps_per_n: float
    max_normal_correction_mps: float
    force_deadband_n: float

    def __post_init__(self) -> None:
        self._validate_nonnegative_finite(self.kp_tangent, "kp_tangent")
        self._validate_nonnegative_finite(self.kd_tangent, "kd_tangent")
        self._validate_nonnegative_finite(
            self.force_gain_mps_per_n, "force_gain_mps_per_n"
        )
        self._validate_nonnegative_finite(
            self.max_normal_correction_mps, "max_normal_correction_mps"
        )
        self._validate_nonnegative_finite(self.force_deadband_n, "force_deadband_n")

    def compute_tool_tip_command(
        self,
        desired_pos_m: ArrayLike,
        desired_vel_mps: ArrayLike,
        current_pos_m: ArrayLike,
        current_vel_mps: ArrayLike,
        surface_tangent: ArrayLike,
        surface_normal: ArrayLike,
        desired_normal_force_n: float,
        measured_normal_force_n: float,
    ) -> np.ndarray:
        """Return a 2D commanded tool-tip velocity in m/s."""

        desired_pos = self._as_vector2(desired_pos_m, "desired_pos_m")
        desired_vel = self._as_vector2(desired_vel_mps, "desired_vel_mps")
        current_pos = self._as_vector2(current_pos_m, "current_pos_m")
        current_vel = self._as_vector2(current_vel_mps, "current_vel_mps")
        tangent = self._as_unit_vector2(surface_tangent, "surface_tangent")
        normal = self._as_unit_vector2(surface_normal, "surface_normal")
        if abs(float(np.dot(tangent, normal))) > _ORTHOGONALITY_TOL:
            raise ValueError("surface_tangent and surface_normal must be orthogonal")

        desired_normal_force = self._finite_scalar(
            desired_normal_force_n, "desired_normal_force_n"
        )
        measured_normal_force = self._finite_scalar(
            measured_normal_force_n, "measured_normal_force_n"
        )

        position_error = desired_pos - current_pos
        velocity_error = desired_vel - current_vel
        tangential_speed_mps = (
            float(np.dot(desired_vel, tangent))
            + self.kp_tangent * float(np.dot(position_error, tangent))
            + self.kd_tangent * float(np.dot(velocity_error, tangent))
        )

        force_error_n = desired_normal_force - measured_normal_force
        effective_force_error_n = self._apply_deadband(
            force_error_n, self.force_deadband_n
        )
        normal_correction_mps = float(
            np.clip(
                self.force_gain_mps_per_n * effective_force_error_n,
                -self.max_normal_correction_mps,
                self.max_normal_correction_mps,
            )
        )

        tangential_command = tangential_speed_mps * tangent
        normal_command = -normal_correction_mps * normal
        return tangential_command + normal_command

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        magnitude = abs(value)
        if magnitude <= deadband:
            return 0.0
        return float(np.sign(value) * (magnitude - deadband))

    @staticmethod
    def _as_vector2(value: ArrayLike, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (2,):
            raise ValueError(f"{name} must have shape (2,)")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array

    @classmethod
    def _as_unit_vector2(cls, value: ArrayLike, name: str) -> np.ndarray:
        vector = cls._as_vector2(value, name)
        norm = float(np.linalg.norm(vector))
        if norm <= 0.0:
            raise ValueError(f"{name} must be nonzero")
        return vector / norm

    @staticmethod
    def _finite_scalar(value: float, name: str) -> float:
        value_float = float(value)
        if not np.isfinite(value_float):
            raise ValueError(f"{name} must be finite")
        return value_float

    @staticmethod
    def _validate_nonnegative_finite(value: float, name: str) -> None:
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be non-negative and finite")
