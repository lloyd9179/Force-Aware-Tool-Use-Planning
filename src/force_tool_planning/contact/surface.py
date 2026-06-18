"""Deterministic 2D surface models for Phase 3 contact execution."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Surface2D:
    """Planar surface model represented by height ``y = h(x)`` in world units.

    The surface tangent points in the positive-x direction. The surface normal
    points away from the surface into free space, which is positive y for a
    flat horizontal surface.
    """

    surface_type: str
    planned_height_m: float
    height_offset_m: float = 0.0
    amplitude_m: float = 0.0
    frequency_cycles_per_m: float = 1.0

    def __post_init__(self) -> None:
        surface_type = self.surface_type.lower()
        if surface_type not in {"flat", "sinusoidal"}:
            raise ValueError("surface_type must be 'flat' or 'sinusoidal'")
        object.__setattr__(self, "surface_type", surface_type)

        values = {
            "planned_height_m": self.planned_height_m,
            "height_offset_m": self.height_offset_m,
            "amplitude_m": self.amplitude_m,
            "frequency_cycles_per_m": self.frequency_cycles_per_m,
        }
        for name, value in values.items():
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.frequency_cycles_per_m < 0.0:
            raise ValueError("frequency_cycles_per_m must be non-negative")

    def height(self, x_m: float) -> float:
        """Return surface height in meters at world x-coordinate ``x_m``."""

        x_value = self._finite_scalar(x_m, "x_m")
        base_height = self.planned_height_m + self.height_offset_m
        if self.surface_type == "flat":
            return float(base_height)

        phase_rad = 2.0 * np.pi * self.frequency_cycles_per_m * x_value
        return float(base_height + self.amplitude_m * np.sin(phase_rad))

    def tangent(self, x_m: float) -> np.ndarray:
        """Return a unit tangent vector at ``x_m`` in the positive-x direction."""

        slope = self._slope(x_m)
        tangent = np.array([1.0, slope], dtype=float)
        return tangent / np.linalg.norm(tangent)

    def normal(self, x_m: float) -> np.ndarray:
        """Return a unit normal vector at ``x_m`` pointing into free space."""

        slope = self._slope(x_m)
        normal = np.array([-slope, 1.0], dtype=float)
        return normal / np.linalg.norm(normal)

    def _slope(self, x_m: float) -> float:
        """Return ``dy/dx`` for the surface at ``x_m``."""

        x_value = self._finite_scalar(x_m, "x_m")
        if self.surface_type == "flat":
            return 0.0

        phase_rad = 2.0 * np.pi * self.frequency_cycles_per_m * x_value
        return float(
            self.amplitude_m
            * 2.0
            * np.pi
            * self.frequency_cycles_per_m
            * np.cos(phase_rad)
        )

    @staticmethod
    def _finite_scalar(value: float, name: str) -> float:
        value_float = float(value)
        if not np.isfinite(value_float):
            raise ValueError(f"{name} must be finite")
        return value_float
