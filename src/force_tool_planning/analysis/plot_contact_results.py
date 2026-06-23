"""Matplotlib plots for Phase 3 contact execution comparisons."""

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
from matplotlib.figure import Figure

from force_tool_planning.analysis.compare_execution import Phase3ComparisonResult
from force_tool_planning.simulation.execution_result import ContactExecutionResult


PHASE3_FIGURE_FILENAMES = {
    "tool_tip_trajectory": "phase3_tool_tip_trajectory.png",
    "force_tracking": "phase3_force_tracking.png",
    "contact_state": "phase3_contact_state.png",
    "torque_ratio": "phase3_torque_ratio.png",
}


def phase3_figure_paths(output_dir: str | Path) -> dict[str, Path]:
    """Return required Phase 3 figure paths under ``output_dir``."""

    directory = Path(output_dir)
    return {
        name: directory / filename
        for name, filename in PHASE3_FIGURE_FILENAMES.items()
    }


def save_phase3_figures(
    comparison: Phase3ComparisonResult,
    output_dir: str | Path,
) -> dict[str, Path]:
    """Save all required Phase 3 comparison figures and return their paths."""

    output_paths = phase3_figure_paths(output_dir)
    plot_tool_tip_trajectory(comparison, output_paths["tool_tip_trajectory"])
    plot_force_tracking(comparison, output_paths["force_tracking"])
    plot_contact_state(comparison, output_paths["contact_state"])
    plot_torque_ratio(comparison, output_paths["torque_ratio"])
    return output_paths


def plot_tool_tip_trajectory(
    comparison: Phase3ComparisonResult,
    output_path: str | Path,
) -> None:
    """Plot desired and actual tool-tip XY paths for both controllers."""

    fig, ax = plt.subplots(figsize=(8, 5.5))
    desired = comparison.force_aware.desired_tool_tip_pos_m
    ax.plot(
        desired[:, 0],
        desired[:, 1],
        linestyle="--",
        linewidth=2,
        color="black",
        label="desired tool-tip path",
    )
    for result in comparison.results:
        actual = result.actual_tool_tip_pos_m
        ax.plot(
            actual[:, 0],
            actual[:, 1],
            linewidth=2,
            label=_controller_label(result),
        )
    ax.set_title("Phase 3 Tool-Tip Trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        ncol=1,
        fontsize=9,
        frameon=True,
        borderaxespad=0.0,
    )
    fig.tight_layout()
    _save_and_close(fig, output_path)


def plot_force_tracking(
    comparison: Phase3ComparisonResult,
    output_path: str | Path,
) -> None:
    """Plot desired and measured normal force over time."""

    fig, ax = plt.subplots(figsize=(9, 5.2))
    desired = comparison.force_aware.desired_normal_force_n
    ax.plot(
        comparison.force_aware.time_s,
        desired,
        linestyle="--",
        linewidth=2,
        color="black",
        label="desired normal force",
    )
    for result in comparison.results:
        ax.plot(
            result.time_s,
            result.normal_force_n,
            linewidth=2,
            label=_controller_label(result),
        )
    ax.set_title("Phase 3 Normal-Force Tracking")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("normal force [N]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    _save_and_close(fig, output_path)


def plot_contact_state(
    comparison: Phase3ComparisonResult,
    output_path: str | Path,
) -> None:
    """Plot binary contact state and penetration depth over time."""

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    contact_ax, penetration_ax = axes
    for result in comparison.results:
        label = _controller_label(result)
        contact_ax.step(
            result.time_s,
            result.is_in_contact.astype(float),
            where="post",
            linewidth=2,
            label=label,
        )
        penetration_ax.plot(
            result.time_s,
            result.penetration_m,
            linewidth=2,
            label=label,
        )

    contact_ax.set_title("Phase 3 Contact State")
    contact_ax.set_ylabel("in contact")
    contact_ax.set_yticks([0.0, 1.0])
    contact_ax.grid(True, alpha=0.3)
    contact_ax.legend()

    penetration_ax.axhline(0.0, color="black", linestyle="--", linewidth=1)
    penetration_ax.set_xlabel("time [s]")
    penetration_ax.set_ylabel("penetration [m]")
    penetration_ax.grid(True, alpha=0.3)
    penetration_ax.legend()
    fig.tight_layout()
    _save_and_close(fig, output_path)


def plot_torque_ratio(
    comparison: Phase3ComparisonResult,
    output_path: str | Path,
) -> None:
    """Plot max joint torque-limit ratio over time."""

    fig, ax = plt.subplots(figsize=(9, 5.2))
    for result in comparison.results:
        ax.plot(
            result.time_s,
            result.torque_ratio,
            linewidth=2,
            label=_controller_label(result),
        )
    ax.axhline(1.0, color="black", linestyle="--", linewidth=1.5, label="limit")
    ax.set_title("Phase 3 Torque Ratio")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("max |tau| / limit")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    _save_and_close(fig, output_path)


def _controller_label(result: ContactExecutionResult) -> str:
    return result.controller_name.replace("_", "-")


def _save_and_close(fig: Figure, output_path: str | Path) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fig.savefig(path, dpi=160, bbox_inches="tight")
    finally:
        plt.close(fig)
