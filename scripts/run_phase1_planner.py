"""Run the deterministic Phase 1 force-aware planner."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

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
from force_tool_planning.planner import plan_force_aware  # noqa: E402
from force_tool_planning.visualization import (  # noqa: E402
    plot_candidate_filtering_summary,
    plot_tool_and_ee_paths,
    plot_torque_profiles,
)


def _display_path(path: Path) -> Path:
    try:
        return path.relative_to(ROOT)
    except ValueError:
        return path


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
    result = plan_force_aware(arm, task, grasps)

    print("=== Phase 1 Force-Aware Planner ===")
    print(f"Task: {task.name}")
    print(f"Path found: {'yes' if result.success else 'no'}")
    print(f"Selected grasp: {result.selected_grasp}")
    print(
        "Torque feasible: "
        f"{'yes' if result.diagnostics.get('torque_feasible') is True else 'no'}"
    )
    max_ratio = "n/a" if result.max_torque_ratio is None else f"{result.max_torque_ratio:.6f}"
    print(f"Max torque ratio: {max_ratio}")
    print(
        "Candidates: "
        f"total={result.total_candidates}, "
        f"joint-feasible={result.joint_limit_feasible_candidates}, "
        f"torque-feasible={result.torque_feasible_candidates}"
    )
    if not result.success:
        print(f"Failure reason: {result.failure_reason}")
        return 1

    output_paths = [
        figures_dir / "tool_and_ee_paths.png",
        figures_dir / "force_aware_torque_profiles.png",
        figures_dir / "force_aware_candidate_filtering_summary.png",
    ]
    plot_tool_and_ee_paths(task, grasps, output_paths[0])
    plot_torque_profiles(arm, None, result, output_paths[1])
    plot_candidate_filtering_summary(None, result, output_paths[2])

    print("Saved figures:")
    for path in output_paths:
        print(f"  {_display_path(path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
