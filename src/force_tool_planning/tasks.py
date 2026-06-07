"""Deterministic planar tool-use task definitions."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ToolUseTask:
    """A planar tool path and desired task wrench.

    The Phase 1 wrench ``[Fx_N, Fy_N, Mz_Nm]`` is applied directly in the
    simplified world/end-effector planar task frame; no wrench transform is
    modeled.
    """

    name: str
    tool_path: np.ndarray
    desired_wrench: np.ndarray

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("task name must be a non-empty string")

        tool_path = np.asarray(self.tool_path, dtype=float)
        if tool_path.ndim != 2 or tool_path.shape[1] != 3:
            raise ValueError("tool_path must have shape (num_waypoints, 3)")
        if tool_path.shape[0] == 0:
            raise ValueError("tool_path must contain at least one waypoint")
        if not np.all(np.isfinite(tool_path)):
            raise ValueError("tool_path must contain only finite values")

        desired_wrench = np.asarray(self.desired_wrench, dtype=float)
        if desired_wrench.shape != (3,):
            raise ValueError("desired_wrench must have shape (3,)")
        if not np.all(np.isfinite(desired_wrench)):
            raise ValueError("desired_wrench must contain only finite values")
        object.__setattr__(self, "tool_path", tool_path.copy())
        object.__setattr__(self, "desired_wrench", desired_wrench.copy())


def make_horizontal_cutting_task(
    *,
    num_waypoints: int = 20,
    start_pose: tuple[float, float, float] = (1.2, 0.6, 0.0),
    length_m: float = 0.5,
    desired_wrench: tuple[float, float, float] = (0.0, -10.0, 0.0),
) -> ToolUseTask:
    """Create a horizontal tool path with a constant simplified planar wrench."""

    if isinstance(num_waypoints, bool) or not isinstance(num_waypoints, int):
        raise ValueError("num_waypoints must be a positive integer")
    if num_waypoints <= 0:
        raise ValueError("num_waypoints must be a positive integer")
    if not np.isfinite(length_m) or length_m < 0.0:
        raise ValueError("length_m must be finite and non-negative")

    start = np.asarray(start_pose, dtype=float)
    if start.shape != (3,):
        raise ValueError("start_pose must have shape (3,)")
    if not np.all(np.isfinite(start)):
        raise ValueError("start_pose must contain only finite values")
    tool_path = np.repeat(start[np.newaxis, :], num_waypoints, axis=0)
    tool_path[:, 0] = np.linspace(start[0], start[0] + length_m, num_waypoints)
    return ToolUseTask(
        name="horizontal_cutting",
        tool_path=tool_path,
        desired_wrench=desired_wrench,
    )
