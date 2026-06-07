from pathlib import Path

from force_tool_planning.config import (
    arm_from_config,
    grasps_from_config,
    load_demo_config,
    task_from_config,
)
from force_tool_planning.planner import plan_baseline, plan_force_aware
from force_tool_planning.visualization import (
    plot_baseline_vs_force_aware,
    plot_candidate_filtering_summary,
    plot_tool_and_ee_paths,
    plot_torque_profiles,
)
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


def test_core_visualizations_save_nonempty_files_and_close_figures(tmp_path: Path) -> None:
    config = load_demo_config(DEFAULT_CONFIG)
    arm = arm_from_config(config)
    task = task_from_config(config)
    grasps = grasps_from_config(config)
    baseline = plan_baseline(arm, task, grasps)
    force_aware = plan_force_aware(arm, task, grasps)
    output_paths = [
        tmp_path / "nested" / "tool_and_ee_paths.png",
        tmp_path / "baseline_vs_force_aware_paths.png",
        tmp_path / "torque_profiles.png",
        tmp_path / "candidate_filtering_summary.png",
    ]

    plot_tool_and_ee_paths(task, grasps, output_paths[0])
    plot_baseline_vs_force_aware(arm, baseline, force_aware, output_paths[1])
    plot_torque_profiles(arm, baseline, force_aware, output_paths[2])
    plot_candidate_filtering_summary(baseline, force_aware, output_paths[3])

    assert all(path.is_file() and path.stat().st_size > 0 for path in output_paths)
    assert plt.get_fignums() == []


def test_force_aware_only_visualizations_save_and_close_figures(tmp_path: Path) -> None:
    config = load_demo_config(DEFAULT_CONFIG)
    arm = arm_from_config(config)
    task = task_from_config(config)
    grasps = grasps_from_config(config)
    force_aware = plan_force_aware(arm, task, grasps)
    torque_path = tmp_path / "force_torque.png"
    candidates_path = tmp_path / "force_candidates.png"

    plot_torque_profiles(arm, None, force_aware, torque_path)
    plot_candidate_filtering_summary(None, force_aware, candidates_path)

    assert torque_path.stat().st_size > 0
    assert candidates_path.stat().st_size > 0
    assert plt.get_fignums() == []
