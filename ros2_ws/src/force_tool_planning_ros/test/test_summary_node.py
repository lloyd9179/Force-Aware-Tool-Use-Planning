"""Tests for the baseline-versus-force-aware terminal summary."""

from pathlib import Path

from force_tool_planning_ros.result_adapter import load_phase1_result_data
from force_tool_planning_ros.summary_node import format_result_summary

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


def test_summary_explains_selected_paths_and_execution_choice() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    summary = "\n".join(format_result_summary(result))

    assert "Baseline: grasp=angled_down, torque_feasible=no" in summary
    assert "Baseline violation waypoints: [0, 1, 2" in summary
    assert "Force-aware: grasp=short_inline, torque_feasible=yes" in summary
    assert "Execution selection: force-aware path only" in summary
