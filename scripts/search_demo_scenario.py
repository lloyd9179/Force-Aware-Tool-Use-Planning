"""Search a small deterministic grid for the intended Phase 1 comparison."""

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
from force_tool_planning.kinematics import ArmModel  # noqa: E402
from force_tool_planning.planner import (  # noqa: E402
    PlanningResult,
    plan_baseline,
    plan_force_aware,
)
from force_tool_planning.tasks import ToolUseTask  # noqa: E402


def intended_demo_condition(
    baseline: PlanningResult,
    force_aware: PlanningResult,
) -> bool:
    """Return whether results demonstrate the intended deterministic story."""

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


def _build_variant(
    base_arm: ArmModel,
    base_task: ToolUseTask,
    *,
    wrench_scale: float,
    torque_limit_scale: float,
    path_y_offset_m: float,
) -> tuple[ArmModel, ToolUseTask]:
    if base_arm.torque_limits_nm is None:
        raise ValueError("base arm must define torque limits")

    arm = ArmModel(
        link_lengths_m=base_arm.link_lengths_m,
        joint_limits_rad=base_arm.joint_limits_rad,
        torque_limits_nm=base_arm.torque_limits_nm * torque_limit_scale,
    )
    tool_path = base_task.tool_path.copy()
    tool_path[:, 1] += path_y_offset_m
    task = ToolUseTask(
        name=base_task.name,
        tool_path=tool_path,
        desired_wrench=base_task.desired_wrench * wrench_scale,
    )
    return arm, task


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs" / "demo_planar_arm.yaml",
        help="Base YAML configuration to search around.",
    )
    args = parser.parse_args()

    config = load_demo_config(args.config)
    base_arm = arm_from_config(config)
    base_task = task_from_config(config)
    grasps = grasps_from_config(config)

    wrench_scales = (1.0, 0.8, 1.2, 1.4)
    torque_limit_scales = (1.0, 0.9, 1.1)
    path_y_offsets_m = (0.0, -0.1, 0.1)

    checked = 0
    for path_y_offset_m in path_y_offsets_m:
        for wrench_scale in wrench_scales:
            for torque_limit_scale in torque_limit_scales:
                checked += 1
                arm, task = _build_variant(
                    base_arm,
                    base_task,
                    wrench_scale=wrench_scale,
                    torque_limit_scale=torque_limit_scale,
                    path_y_offset_m=path_y_offset_m,
                )
                baseline = plan_baseline(arm, task, grasps)
                force_aware = plan_force_aware(arm, task, grasps)
                if not intended_demo_condition(baseline, force_aware):
                    continue

                print("Found deterministic Phase 1 scenario")
                print(f"  checked scenarios: {checked}")
                print(f"  wrench scale: {wrench_scale}")
                print(f"  torque limit scale: {torque_limit_scale}")
                print(f"  path y offset [m]: {path_y_offset_m}")
                print(
                    "  baseline: "
                    f"grasp={baseline.selected_grasp}, "
                    f"max torque ratio={baseline.max_torque_ratio:.6f}"
                )
                print(
                    "  force-aware: "
                    f"grasp={force_aware.selected_grasp}, "
                    f"max torque ratio={force_aware.max_torque_ratio:.6f}"
                )
                return 0

    print(f"No intended Phase 1 scenario found after checking {checked} variants")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
