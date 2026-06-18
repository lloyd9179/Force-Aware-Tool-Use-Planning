"""Position-only tool-tip controller for Phase 3 baseline execution."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class PositionOnlyController:
    """Track desired 2D tool-tip motion without using force feedback.

    The command is a velocity command:

    ``desired_vel + kp_task * position_error + kd_task * velocity_error``

    This keeps the baseline purely kinematic and position-based for the Phase 3
    contact execution comparison.
    """

    kp_task: float
    kd_task: float

    def __post_init__(self) -> None:
        if not np.isfinite(self.kp_task) or self.kp_task < 0.0:
            raise ValueError("kp_task must be non-negative and finite")
        if not np.isfinite(self.kd_task) or self.kd_task < 0.0:
            raise ValueError("kd_task must be non-negative and finite")

    def compute_tool_tip_command(
        self,
        desired_pos_m: ArrayLike,
        desired_vel_mps: ArrayLike,
        current_pos_m: ArrayLike,
        current_vel_mps: ArrayLike,
    ) -> np.ndarray:
        """Return a 2D commanded tool-tip velocity in m/s.

        This method intentionally has no measured-force argument. Force-aware
        corrections belong in `ForceAwareController`.
        """

        desired_pos = self._as_vector2(desired_pos_m, "desired_pos_m")
        desired_vel = self._as_vector2(desired_vel_mps, "desired_vel_mps")
        current_pos = self._as_vector2(current_pos_m, "current_pos_m")
        current_vel = self._as_vector2(current_vel_mps, "current_vel_mps")

        position_error = desired_pos - current_pos
        velocity_error = desired_vel - current_vel
        return desired_vel + self.kp_task * position_error + self.kd_task * velocity_error

    @staticmethod
    def _as_vector2(value: ArrayLike, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (2,):
            raise ValueError(f"{name} must have shape (2,)")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array
