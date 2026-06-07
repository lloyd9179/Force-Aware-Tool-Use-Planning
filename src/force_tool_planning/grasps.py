"""Simplified planar grasp transforms for the Phase 1 planning demo."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from force_tool_planning.transforms import as_pose3, transform_path


@dataclass(frozen=True)
class Grasp:
    """A simplified rigid grasp transform from the tool frame to the EE frame.

    ``tool_T_ee`` is the end-effector pose expressed in the tool frame. Phase 1
    treats the tool-tip and tool frames as the same frame.
    """

    name: str
    tool_T_ee: np.ndarray

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("grasp name must be a non-empty string")
        object.__setattr__(self, "tool_T_ee", as_pose3(self.tool_T_ee).copy())


def make_default_grasps() -> list[Grasp]:
    """Return the deterministic simplified grasp set used by Phase 1."""

    return [
        Grasp(name="short_inline", tool_T_ee=[-0.20, 0.00, 0.00]),
        Grasp(name="long_inline", tool_T_ee=[-0.60, 0.00, 0.00]),
        Grasp(name="angled_up", tool_T_ee=[-0.40, 0.00, 0.60]),
        Grasp(name="angled_down", tool_T_ee=[-0.40, 0.00, -0.60]),
    ]


def tool_path_to_ee_path(tool_path: np.ndarray, grasp: Grasp) -> np.ndarray:
    """Return ``world_T_ee = world_T_tool compose tool_T_ee`` at each waypoint."""

    return transform_path(tool_path, grasp.tool_T_ee)
