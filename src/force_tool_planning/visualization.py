"""Non-interactive Matplotlib visualizations for the Phase 1 demo."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

_MPL_CONFIG_DIR = Path(tempfile.gettempdir()) / "force-aware-tool-use-planning-matplotlib"
_MPL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_MPL_CONFIG_DIR))

import matplotlib

matplotlib.use("Agg", force=True)

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from force_tool_planning.grasps import Grasp, tool_path_to_ee_path
from force_tool_planning.kinematics import ArmModel, joint_positions
from force_tool_planning.planner import PlanningResult
from force_tool_planning.tasks import ToolUseTask


def _save_and_close(fig: Figure, output_path: str | Path) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fig.savefig(path, dpi=160, bbox_inches="tight")
    finally:
        plt.close(fig)


def _require_successful_path(result: PlanningResult, label: str) -> None:
    if not result.success or result.path_q is None:
        raise ValueError(f"{label} result must contain a successful joint path")


def plot_tool_and_ee_paths(
    task: ToolUseTask,
    grasps: list[Grasp],
    output_path: str | Path,
) -> None:
    """Plot ``world_T_tool`` and the ``world_T_ee`` path induced by each grasp."""

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    path_ax, orientation_ax = axes
    path_ax.plot(
        task.tool_path[:, 0],
        task.tool_path[:, 1],
        marker="o",
        markersize=3,
        linewidth=2,
        label="tool path",
    )
    waypoint_indices = np.arange(len(task.tool_path))
    orientation_ax.plot(
        waypoint_indices,
        task.tool_path[:, 2],
        linewidth=2,
        label="tool orientation",
    )
    for grasp in grasps:
        ee_path = tool_path_to_ee_path(task.tool_path, grasp)
        path_ax.plot(
            ee_path[:, 0],
            ee_path[:, 1],
            marker=".",
            linewidth=1.5,
            label=f"EE: {grasp.name}",
        )
        orientation_ax.plot(
            waypoint_indices,
            ee_path[:, 2],
            linewidth=1.5,
            label=f"EE: {grasp.name}",
        )

    path_ax.set_title("World-Frame XY Paths")
    path_ax.set_xlabel("world x [m]")
    path_ax.set_ylabel("world y [m]")
    path_ax.set_aspect("equal", adjustable="box")
    path_ax.grid(True, alpha=0.3)
    path_ax.legend(fontsize="small")
    orientation_ax.set_title("Path Orientation")
    orientation_ax.set_xlabel("waypoint index")
    orientation_ax.set_ylabel("theta [rad]")
    orientation_ax.grid(True, alpha=0.3)
    orientation_ax.legend(fontsize="small")
    fig.suptitle("Tool Path and Grasp-Induced End-Effector Paths")
    fig.tight_layout()
    _save_and_close(fig, output_path)


def _plot_arm_trajectory(
    ax: Axes,
    arm: ArmModel,
    result: PlanningResult,
    *,
    title: str,
) -> None:
    _require_successful_path(result, title)
    assert result.path_q is not None

    sample_indices = np.unique(
        np.linspace(0, len(result.path_q) - 1, min(6, len(result.path_q)), dtype=int)
    )
    for sample_index in sample_indices:
        positions_m = joint_positions(arm, result.path_q[sample_index])
        ax.plot(
            positions_m[:, 0],
            positions_m[:, 1],
            marker="o",
            linewidth=1.2,
            alpha=0.45,
        )

    ee_positions_m = np.asarray(
        [joint_positions(arm, q)[-1] for q in result.path_q],
        dtype=float,
    )
    ratio_text = (
        "n/a" if result.max_torque_ratio is None else f"{result.max_torque_ratio:.3f}"
    )
    ax.plot(
        ee_positions_m[:, 0],
        ee_positions_m[:, 1],
        linewidth=2.5,
        label="selected EE path",
    )
    ax.plot(
        result.tool_path[:, 0],
        result.tool_path[:, 1],
        linestyle="--",
        linewidth=1.5,
        label="tool path",
    )
    ax.scatter([0.0], [0.0], marker="s", label="robot base")
    ax.set_title(
        f"{title}\n"
        f"grasp={result.selected_grasp}, max torque ratio={ratio_text}"
    )
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend()


def plot_baseline_vs_force_aware(
    arm: ArmModel,
    baseline_result: PlanningResult,
    force_result: PlanningResult,
    output_path: str | Path,
) -> None:
    """Plot sampled arm configurations for baseline and force-aware paths."""

    fig, axes = plt.subplots(1, 2, figsize=(13, 6), sharex=True, sharey=True)
    _plot_arm_trajectory(axes[0], arm, baseline_result, title="Baseline")
    _plot_arm_trajectory(axes[1], arm, force_result, title="Force-aware")
    fig.suptitle("Baseline vs Force-Aware Selected Arm Paths")
    fig.tight_layout()
    _save_and_close(fig, output_path)


def _plot_result_torques(
    ax: Axes,
    result: PlanningResult,
    torque_limits_nm: np.ndarray,
    *,
    title: str,
) -> None:
    _require_successful_path(result, title)
    torque_checks = [candidate.torque_check for candidate in result.selected_candidates]
    if any(check is None for check in torque_checks):
        raise ValueError(f"{title} result selected path has no torque diagnostics")

    torque_magnitudes_nm = np.asarray(
        [np.abs(check.torque_nm) for check in torque_checks if check is not None],
        dtype=float,
    )
    waypoint_indices = np.arange(torque_magnitudes_nm.shape[0])
    for joint_index in range(torque_magnitudes_nm.shape[1]):
        line = ax.plot(
            waypoint_indices,
            torque_magnitudes_nm[:, joint_index],
            linewidth=2,
            label=f"|tau{joint_index + 1}|",
        )[0]
        ax.axhline(
            torque_limits_nm[joint_index],
            color=line.get_color(),
            linestyle="--",
            linewidth=1,
            label=f"joint {joint_index + 1} limit",
        )
    ax.set_title(title)
    ax.set_xlabel("waypoint index")
    ax.set_ylabel("joint torque magnitude [N m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small")


def plot_torque_profiles(
    arm: ArmModel,
    baseline_result: PlanningResult | None,
    force_result: PlanningResult | None,
    output_path: str | Path,
) -> None:
    """Plot selected-path joint torque magnitudes with joint torque limits."""

    if arm.torque_limits_nm is None:
        raise ValueError("arm.torque_limits_nm is required for torque profiles")
    results = [
        (label, result)
        for label, result in (
            ("Baseline", baseline_result),
            ("Force-aware", force_result),
        )
        if result is not None
    ]
    if not results:
        raise ValueError("at least one planning result is required")

    fig, axes = plt.subplots(
        len(results),
        1,
        figsize=(10, 4.5 * len(results)),
        squeeze=False,
        sharex=len(results) > 1,
    )
    for ax, (label, result) in zip(axes[:, 0], results):
        _plot_result_torques(
            ax,
            result,
            arm.torque_limits_nm,
            title=f"{label} Selected-Path Torque Profile",
        )
    fig.tight_layout()
    _save_and_close(fig, output_path)


def plot_candidate_filtering_summary(
    baseline_result: PlanningResult | None,
    force_result: PlanningResult | None,
    output_path: str | Path,
) -> None:
    """Plot candidate counts and actual rejection counts for each planner."""

    results = [
        result for result in (baseline_result, force_result) if result is not None
    ]
    if not results:
        raise ValueError("at least one planning result is required")

    labels = [result.planner_name.replace("_", "-") for result in results]
    total = np.asarray([result.total_candidates for result in results], dtype=float)
    joint_rejected = np.asarray(
        [len(result.rejected_by_joint_limits) for result in results],
        dtype=float,
    )
    torque_rejected = np.asarray(
        [len(result.rejected_by_torque) for result in results],
        dtype=float,
    )
    accepted = total - joint_rejected - torque_rejected

    x = np.arange(len(results))
    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.bar(x, accepted, label="accepted by planner filters")
    ax.bar(x, joint_rejected, bottom=accepted, label="rejected by joint limits")
    ax.bar(
        x,
        torque_rejected,
        bottom=accepted + joint_rejected,
        label="rejected by torque limits",
    )
    ax.set_xticks(x, labels)
    ax.set_ylabel("candidate count")
    ax.set_title("Candidate Filtering Summary")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    _save_and_close(fig, output_path)
