"""Tests for the deterministic Phase 1-to-ROS result adapter."""

from pathlib import Path

import numpy as np

from force_tool_planning_ros.result_adapter import load_phase1_result_data

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


def test_result_contract_shapes_and_frames() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    assert result.frame_id == "base_link"
    assert result.joint_names == ("joint1", "joint2", "joint3")
    assert result.tool_path.shape == (20, 3)
    assert result.desired_wrench.shape == (3,)

    for planner_path in (result.baseline, result.force_aware):
        assert planner_path.joint_path.shape == (20, 3)
        assert planner_path.ee_path.shape == (20, 3)
        assert planner_path.joint_torques_nm.shape == (20, 3)
        assert planner_path.tool_T_ee.shape == (3,)
        assert np.all(np.isfinite(planner_path.joint_path))
        assert np.all(np.isfinite(planner_path.joint_torques_nm))


def test_result_contract_preserves_selected_grasps_and_feasibility() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    assert result.baseline.planner_name == "baseline"
    assert result.baseline.selected_grasp == "angled_down"
    assert np.allclose(result.baseline.tool_T_ee, [-0.40, 0.00, -0.60])
    assert result.baseline.torque_feasible is False
    assert result.baseline.max_torque_ratio > 1.0

    assert result.force_aware.planner_name == "force_aware"
    assert result.force_aware.selected_grasp == "short_inline"
    assert np.allclose(result.force_aware.tool_T_ee, [-0.20, 0.00, 0.00])
    assert result.force_aware.torque_feasible is True
    assert result.force_aware.max_torque_ratio <= 1.0


def test_result_contract_derives_baseline_violation_waypoints() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    assert result.baseline_violation_waypoint_indices == tuple(range(20))
    assert all(
        np.any(
            np.abs(result.baseline.joint_torques_nm[waypoint_index])
            > np.array([18.0, 12.0, 8.0])
        )
        for waypoint_index in result.baseline_violation_waypoint_indices
    )
    assert not any(
        np.any(np.abs(torque_nm) > np.array([18.0, 12.0, 8.0]))
        for torque_nm in result.force_aware.joint_torques_nm
    )
