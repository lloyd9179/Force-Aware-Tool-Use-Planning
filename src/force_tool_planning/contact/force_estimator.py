"""Contact force-to-wrench helpers for Phase 3 execution demos."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


def estimate_contact_wrench_2d(
    normal_force_n: float,
    tangential_force_n: float = 0.0,
    contact_point_m: ArrayLike | None = None,
) -> np.ndarray:
    """Return planar contact wrench ``[Fx, Fy, tau_z]``.

    The wrench is expressed in the same simplified planar convention used by
    the Phase 1 torque utility. ``normal_force_n`` acts along positive y,
    ``tangential_force_n`` acts along positive x, and ``contact_point_m`` is the
    2D moment arm from the frame origin to the contact point.
    """

    normal_force = _finite_scalar(normal_force_n, "normal_force_n")
    tangential_force = _finite_scalar(tangential_force_n, "tangential_force_n")
    contact_point = _contact_point_or_origin(contact_point_m)

    torque_z_nm = contact_point[0] * normal_force - contact_point[1] * tangential_force
    return np.array([tangential_force, normal_force, torque_z_nm], dtype=float)


def _contact_point_or_origin(contact_point_m: ArrayLike | None) -> np.ndarray:
    if contact_point_m is None:
        return np.zeros(2, dtype=float)

    contact_point = np.asarray(contact_point_m, dtype=float)
    if contact_point.shape != (2,):
        raise ValueError("contact_point_m must have shape (2,)")
    if not np.all(np.isfinite(contact_point)):
        raise ValueError("contact_point_m must contain only finite values")
    return contact_point


def _finite_scalar(value: float, name: str) -> float:
    value_float = float(value)
    if not np.isfinite(value_float):
        raise ValueError(f"{name} must be finite")
    return value_float
