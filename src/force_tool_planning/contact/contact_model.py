"""Simplified planar contact model for Phase 3 execution demos."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from numpy.typing import ArrayLike

from force_tool_planning.contact.surface import Surface2D


@dataclass(frozen=True)
class ContactState:
    """Structured result from the simplified tool-tip contact model."""

    surface_height_m: float
    penetration_m: float
    normal_force_n: float
    friction_limit_n: float
    is_in_contact: bool
    is_excessive_penetration: bool

    def as_dict(self) -> dict[str, float | bool]:
        """Return a dictionary representation for scripts and diagnostics."""

        return asdict(self)


@dataclass(frozen=True)
class PlanarContactModel:
    """Linear spring-damper contact model for a 2D tool tip and surface."""

    normal_stiffness_n_per_m: float
    normal_damping_n_s_per_m: float
    friction_coefficient: float
    max_penetration_m: float

    def __post_init__(self) -> None:
        if not np.isfinite(self.normal_stiffness_n_per_m) or self.normal_stiffness_n_per_m <= 0.0:
            raise ValueError("normal_stiffness_n_per_m must be positive and finite")
        if not np.isfinite(self.normal_damping_n_s_per_m) or self.normal_damping_n_s_per_m < 0.0:
            raise ValueError("normal_damping_n_s_per_m must be non-negative and finite")
        if not np.isfinite(self.friction_coefficient) or self.friction_coefficient < 0.0:
            raise ValueError("friction_coefficient must be non-negative and finite")
        if not np.isfinite(self.max_penetration_m) or self.max_penetration_m < 0.0:
            raise ValueError("max_penetration_m must be non-negative and finite")

    def compute_contact(
        self,
        tool_tip_pos_m: ArrayLike,
        tool_tip_vel_mps: ArrayLike,
        surface: Surface2D,
    ) -> ContactState:
        """Compute simplified contact state for a tool tip against ``surface``.

        Penetration follows the Phase 3 vertical height-field convention:
        ``penetration = surface_height - tool_tip_y``. A positive value means
        the tool tip is below the surface height and contact is active.
        """

        pos = self._as_vector2(tool_tip_pos_m, "tool_tip_pos_m")
        vel = self._as_vector2(tool_tip_vel_mps, "tool_tip_vel_mps")

        surface_height_m = surface.height(float(pos[0]))
        penetration_m = float(surface_height_m - pos[1])
        is_in_contact = penetration_m > 0.0
        is_excessive_penetration = penetration_m > self.max_penetration_m

        if not is_in_contact:
            return ContactState(
                surface_height_m=float(surface_height_m),
                penetration_m=float(penetration_m),
                normal_force_n=0.0,
                friction_limit_n=0.0,
                is_in_contact=False,
                is_excessive_penetration=is_excessive_penetration,
            )

        normal_velocity_mps = float(np.dot(vel, surface.normal(float(pos[0]))))
        normal_force_n = max(
            0.0,
            self.normal_stiffness_n_per_m * penetration_m
            - self.normal_damping_n_s_per_m * normal_velocity_mps,
        )
        friction_limit_n = self.friction_coefficient * normal_force_n

        return ContactState(
            surface_height_m=float(surface_height_m),
            penetration_m=penetration_m,
            normal_force_n=float(normal_force_n),
            friction_limit_n=float(friction_limit_n),
            is_in_contact=True,
            is_excessive_penetration=is_excessive_penetration,
        )

    @staticmethod
    def _as_vector2(value: ArrayLike, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (2,):
            raise ValueError(f"{name} must have shape (2,)")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array
