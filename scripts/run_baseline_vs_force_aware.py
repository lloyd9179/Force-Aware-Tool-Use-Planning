"""Run the deterministic Phase 1 baseline versus force-aware demo."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from force_tool_planning.config import (  # noqa: E402
    arm_from_config,
    grasps_from_config,
    load_demo_config,
    task_from_config,
)
from force_tool_planning.planner import (  # noqa: E402
    PlanningResult,
    plan_baseline,
    plan_force_aware,
)
from force_tool_planning.visualization import (  # noqa: E402
    plot_baseline_vs_force_aware,
    plot_candidate_filtering_summary,
    plot_tool_and_ee_paths,
    plot_torque_profiles,
)


def _yes_no(value: object) -> str:
    return "yes" if value is True else "no"


def _ratio_text(result: PlanningResult) -> str:
    return "n/a" if result.max_torque_ratio is None else f"{result.max_torque_ratio:.6f}"


def _display_path(path: Path) -> Path:
    try:
        return path.relative_to(ROOT)
    except ValueError:
        return path


def _intended_demo_condition(
    baseline: PlanningResult,
    force_aware: PlanningResult,
) -> bool:
    if not baseline.success or not force_aware.success:
        return False
    if baseline.diagnostics.get("torque_feasible") is not False:
        return False
    if force_aware.diagnostics.get("torque_feasible") is not True:
        return False
    if baseline.selected_grasp != force_aware.selected_grasp:
        return True
    if baseline.path_q is None or force_aware.path_q is None:
        return False
    return not np.allclose(baseline.path_q, force_aware.path_q)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs" / "demo_planar_arm.yaml",
        help="YAML configuration for the deterministic demo.",
    )
    args = parser.parse_args()

    config = load_demo_config(args.config)
    arm = arm_from_config(config)
    task = task_from_config(config)
    grasps = grasps_from_config(config)
    figures_dir = ROOT / config["output"]["figures_dir"]

    baseline = plan_baseline(arm, task, grasps)
    force_aware = plan_force_aware(arm, task, grasps)

    print("=== Force-Aware Tool-Use Planning: Phase 1 Demo ===")
    print()
    print(f"Task: {task.name}")
    print(f"Waypoints: {len(task.tool_path)}")
    print(f"Desired wrench [Fx, Fy, Mz]: {task.desired_wrench.tolist()}")
    print(f"Candidate grasps: {', '.join(grasp.name for grasp in grasps)}")
    print()
    print("Baseline planner:")
    print(f"  geometric path found: {_yes_no(baseline.success)}")
    print(f"  selected grasp: {baseline.selected_grasp}")
    print(
        "  torque feasible after evaluation: "
        f"{_yes_no(baseline.diagnostics.get('torque_feasible'))}"
    )
    print(f"  max torque ratio: {_ratio_text(baseline)}")
    print(f"  violating joints: {baseline.diagnostics.get('violating_joint_indices', [])}")
    if not baseline.success:
        print(f"  failure reason: {baseline.failure_reason}")
    print()
    print("Force-aware planner:")
    print(f"  path found: {_yes_no(force_aware.success)}")
    print(f"  selected grasp: {force_aware.selected_grasp}")
    print(f"  torque feasible: {_yes_no(force_aware.diagnostics.get('torque_feasible'))}")
    print(f"  max torque ratio: {_ratio_text(force_aware)}")
    if not force_aware.success:
        print(f"  failure reason: {force_aware.failure_reason}")

    if not baseline.success or not force_aware.success:
        return 1

    output_paths = [
        figures_dir / "tool_and_ee_paths.png",
        figures_dir / "baseline_vs_force_aware_paths.png",
        figures_dir / "torque_profiles.png",
        figures_dir / "candidate_filtering_summary.png",
    ]
    plot_tool_and_ee_paths(task, grasps, output_paths[0])
    plot_baseline_vs_force_aware(arm, baseline, force_aware, output_paths[1])
    plot_torque_profiles(arm, baseline, force_aware, output_paths[2])
    plot_candidate_filtering_summary(baseline, force_aware, output_paths[3])

    print()
    print("Saved figures:")
    for path in output_paths:
        print(f"  {_display_path(path)}")

    intended_condition = _intended_demo_condition(baseline, force_aware)
    print()
    print(f"Intended demo condition met: {_yes_no(intended_condition)}")
    return 0 if intended_condition else 1


if __name__ == "__main__":
    raise SystemExit(main())
