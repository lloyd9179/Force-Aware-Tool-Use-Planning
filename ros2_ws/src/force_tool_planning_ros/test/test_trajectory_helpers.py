"""Tests for deterministic force-aware JointTrajectory conversion."""

from pathlib import Path

import numpy as np
import pytest

from force_tool_planning_ros.result_adapter import load_phase1_result_data
from force_tool_planning_ros.trajectory_helpers import (
    build_execution_trajectories,
    build_force_aware_execution_trajectories,
    build_force_aware_joint_trajectory,
    build_move_to_first_waypoint_trajectory,
    build_selected_joint_trajectory,
)

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


def duration_seconds(duration) -> float:
    """Return a builtin_interfaces Duration as seconds."""

    return duration.sec + duration.nanosec / 1_000_000_000


def test_force_aware_trajectory_preserves_joint_names_and_positions() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    trajectory = build_force_aware_joint_trajectory(result)

    assert trajectory.joint_names == ["joint1", "joint2", "joint3"]
    assert len(trajectory.points) == len(result.force_aware.joint_path) == 20
    assert np.allclose(
        [point.positions for point in trajectory.points],
        result.force_aware.joint_path,
    )
    assert not np.allclose(
        [point.positions for point in trajectory.points],
        result.baseline.joint_path,
    )
    assert all(not point.velocities for point in trajectory.points)
    assert all(not point.accelerations for point in trajectory.points)
    assert all(not point.effort for point in trajectory.points)


def test_force_aware_trajectory_uses_deterministic_positive_timestamps() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    trajectory = build_force_aware_joint_trajectory(
        result,
        waypoint_interval_s=0.25,
    )

    timestamps = [
        duration_seconds(point.time_from_start)
        for point in trajectory.points
    ]
    assert timestamps == pytest.approx(
        [0.25 * index for index in range(1, 21)]
    )
    assert all(
        later > earlier
        for earlier, later in zip(timestamps, timestamps[1:])
    )


def test_first_waypoint_move_contains_only_the_planned_start() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    trajectory = build_move_to_first_waypoint_trajectory(
        result,
        move_duration_s=3.0,
    )

    assert trajectory.joint_names == ["joint1", "joint2", "joint3"]
    assert len(trajectory.points) == 1
    assert np.allclose(
        trajectory.points[0].positions,
        result.force_aware.joint_path[0],
    )
    assert duration_seconds(trajectory.points[0].time_from_start) == 3.0


def test_execution_sequence_positions_first_then_follows_selected_path() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    first_move, selected_path = build_force_aware_execution_trajectories(
        result,
        first_waypoint_move_duration_s=3.0,
        waypoint_interval_s=0.5,
    )

    assert len(first_move.points) == 1
    assert len(selected_path.points) == 20
    assert np.allclose(
        first_move.points[0].positions,
        selected_path.points[0].positions,
    )
    assert duration_seconds(first_move.points[0].time_from_start) == 3.0
    assert duration_seconds(selected_path.points[0].time_from_start) == 0.5


def test_baseline_comparison_trajectory_uses_torque_infeasible_path() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    first_move, selected_path = build_execution_trajectories(
        result,
        "baseline",
    )

    assert result.baseline.torque_feasible is False
    assert np.allclose(
        first_move.points[0].positions,
        result.baseline.joint_path[0],
    )
    assert np.allclose(
        [point.positions for point in selected_path.points],
        result.baseline.joint_path,
    )
    assert not np.allclose(
        [point.positions for point in selected_path.points],
        result.force_aware.joint_path,
    )


def test_selected_trajectory_rejects_unknown_planner() -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    with pytest.raises(ValueError, match="planner name must be one of"):
        build_selected_joint_trajectory(result, "unknown")


@pytest.mark.parametrize("waypoint_interval_s", [0.0, -0.5, np.inf, np.nan])
def test_force_aware_trajectory_rejects_invalid_waypoint_intervals(
    waypoint_interval_s: float,
) -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    with pytest.raises(ValueError, match="finite and positive"):
        build_force_aware_joint_trajectory(result, waypoint_interval_s)
