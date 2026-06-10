"""Adapt deterministic Phase 1 planning results for Phase 2 ROS consumers.

The data contract intentionally contains no ROS message types. All planar poses
use ``[x_m, y_m, theta_rad]`` in ``base_link`` and all joint paths use the
ordered joints ``joint1``, ``joint2``, and ``joint3``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from force_tool_planning.config import (
    arm_from_config,
    grasps_from_config,
    load_demo_config,
    task_from_config,
)
from force_tool_planning.grasps import Grasp
from force_tool_planning.planner import (
    IKCandidate,
    PlanningResult,
    plan_baseline,
    plan_force_aware,
)

FRAME_ID = "base_link"
JOINT_NAMES = ("joint1", "joint2", "joint3")


@dataclass(frozen=True)
class PlannerPathData:
    """One selected Phase 1 path prepared for Phase 2 consumers."""

    planner_name: str
    selected_grasp: str
    tool_T_ee: np.ndarray
    joint_path: np.ndarray
    ee_path: np.ndarray
    joint_torques_nm: np.ndarray
    torque_feasible: bool
    max_torque_ratio: float


@dataclass(frozen=True)
class Phase1ResultData:
    """Complete Phase 1 comparison consumed by Phase 2 ROS nodes.

    ``baseline_violation_waypoint_indices`` identifies selected baseline
    waypoints where at least one joint exceeds its torque limit.
    """

    frame_id: str
    joint_names: tuple[str, str, str]
    tool_path: np.ndarray
    desired_wrench: np.ndarray
    baseline: PlannerPathData
    force_aware: PlannerPathData
    baseline_violation_waypoint_indices: tuple[int, ...]


def _selected_torque_data(
    selected_candidates: list[IKCandidate],
    expected_waypoints: int,
) -> tuple[np.ndarray, bool]:
    if len(selected_candidates) != expected_waypoints:
        raise ValueError(
            "selected candidate count must match the path waypoint count"
        )
    if any(
        candidate.torque_check is None
        for candidate in selected_candidates
    ):
        raise ValueError(
            "every selected candidate must include a torque check"
        )

    torque_checks = [
        candidate.torque_check for candidate in selected_candidates
    ]
    joint_torques_nm = np.asarray(
        [check.torque_nm for check in torque_checks if check is not None],
        dtype=float,
    )
    if joint_torques_nm.shape != (expected_waypoints, len(JOINT_NAMES)):
        raise ValueError(
            "selected joint torque data must have shape "
            f"({expected_waypoints}, {len(JOINT_NAMES)})"
        )
    return joint_torques_nm, all(
        check.feasible for check in torque_checks if check is not None
    )


def _grasp_by_name(grasps: list[Grasp], selected_grasp: str) -> Grasp:
    matches = [grasp for grasp in grasps if grasp.name == selected_grasp]
    if len(matches) != 1:
        raise ValueError(
            f"selected grasp must match exactly one grasp: {selected_grasp}"
        )
    return matches[0]


def _adapt_selected_path(
    result: PlanningResult,
    grasps: list[Grasp],
    expected_waypoints: int,
) -> PlannerPathData:
    if not result.success:
        raise ValueError(
            f"{result.planner_name} planning result must be successful"
        )
    if (
        result.selected_grasp is None
        or result.path_q is None
        or result.ee_path is None
    ):
        raise ValueError(
            f"{result.planner_name} result is missing its selected path"
        )
    if result.max_torque_ratio is None:
        raise ValueError(
            f"{result.planner_name} result is missing its torque ratio"
        )

    joint_path = np.asarray(result.path_q, dtype=float)
    ee_path = np.asarray(result.ee_path, dtype=float)
    expected_shape = (expected_waypoints, len(JOINT_NAMES))
    if joint_path.shape != expected_shape:
        raise ValueError(
            f"{result.planner_name} joint path must have shape "
            f"{expected_shape}"
        )
    if ee_path.shape != (expected_waypoints, 3):
        raise ValueError(
            f"{result.planner_name} end-effector path must have shape "
            f"({expected_waypoints}, 3)"
        )

    joint_torques_nm, torque_feasible = _selected_torque_data(
        result.selected_candidates,
        expected_waypoints,
    )
    return PlannerPathData(
        planner_name=result.planner_name,
        selected_grasp=result.selected_grasp,
        tool_T_ee=_grasp_by_name(
            grasps,
            result.selected_grasp,
        ).tool_T_ee.copy(),
        joint_path=joint_path.copy(),
        ee_path=ee_path.copy(),
        joint_torques_nm=joint_torques_nm,
        torque_feasible=torque_feasible,
        max_torque_ratio=float(result.max_torque_ratio),
    )


def adapt_phase1_results(
    baseline: PlanningResult,
    force_aware: PlanningResult,
    grasps: list[Grasp],
) -> Phase1ResultData:
    """Return validated, ROS-facing copies of the selected Phase 1 results."""

    tool_path = np.asarray(baseline.tool_path, dtype=float)
    desired_wrench = np.asarray(baseline.desired_wrench, dtype=float)
    if tool_path.ndim != 2 or tool_path.shape[1] != 3:
        raise ValueError("tool path must have shape (num_waypoints, 3)")
    if desired_wrench.shape != (3,):
        raise ValueError("desired wrench must have shape (3,)")
    if not np.array_equal(force_aware.tool_path, tool_path):
        raise ValueError(
            "baseline and force-aware results must use the same tool path"
        )
    if not np.array_equal(force_aware.desired_wrench, desired_wrench):
        raise ValueError(
            "baseline and force-aware results must use the same wrench"
        )

    expected_waypoints = len(tool_path)
    baseline_data = _adapt_selected_path(baseline, grasps, expected_waypoints)
    force_aware_data = _adapt_selected_path(
        force_aware,
        grasps,
        expected_waypoints,
    )
    violation_waypoints = tuple(
        candidate.waypoint_index
        for candidate in baseline.selected_candidates
        if candidate.torque_check is not None
        and not candidate.torque_check.feasible
    )

    return Phase1ResultData(
        frame_id=FRAME_ID,
        joint_names=JOINT_NAMES,
        tool_path=tool_path.copy(),
        desired_wrench=desired_wrench.copy(),
        baseline=baseline_data,
        force_aware=force_aware_data,
        baseline_violation_waypoint_indices=violation_waypoints,
    )


def load_phase1_result_data(config_path: str | Path) -> Phase1ResultData:
    """Run Phase 1 and return the deterministic Phase 2 data contract."""

    config = load_demo_config(config_path)
    arm = arm_from_config(config)
    task = task_from_config(config)
    grasps = grasps_from_config(config)
    baseline = plan_baseline(arm, task, grasps)
    force_aware = plan_force_aware(arm, task, grasps)
    return adapt_phase1_results(baseline, force_aware, grasps)
