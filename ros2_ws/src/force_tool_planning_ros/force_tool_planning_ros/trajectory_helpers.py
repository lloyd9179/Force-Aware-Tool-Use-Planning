"""Convert selected Phase 1 paths into ROS trajectory messages."""

from __future__ import annotations

import math

import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from force_tool_planning_ros.result_adapter import (
    Phase1ResultData,
    PlannerPathData,
)

DEFAULT_WAYPOINT_INTERVAL_S = 0.5
DEFAULT_FIRST_WAYPOINT_MOVE_DURATION_S = 3.0
NANOSECONDS_PER_SECOND = 1_000_000_000
PLANNER_NAMES = ("force_aware", "baseline")


def _duration_from_seconds(seconds: float) -> Duration:
    total_nanoseconds = round(seconds * NANOSECONDS_PER_SECOND)
    duration = Duration()
    duration.sec, duration.nanosec = divmod(
        total_nanoseconds,
        NANOSECONDS_PER_SECOND,
    )
    return duration


def selected_planner_path(
    result: Phase1ResultData,
    planner_name: str,
) -> PlannerPathData:
    """Return one explicitly selected Phase 1 planner path."""

    if planner_name == "force_aware":
        return result.force_aware
    if planner_name == "baseline":
        return result.baseline
    raise ValueError(
        f"planner name must be one of {PLANNER_NAMES}: {planner_name}"
    )


def build_selected_joint_trajectory(
    result: Phase1ResultData,
    planner_name: str,
    waypoint_interval_s: float = DEFAULT_WAYPOINT_INTERVAL_S,
) -> JointTrajectory:
    """Return a position trajectory for one explicitly selected planner."""

    if not math.isfinite(waypoint_interval_s) or waypoint_interval_s <= 0.0:
        raise ValueError("waypoint interval must be finite and positive")
    planner_path = selected_planner_path(result, planner_name)
    if planner_name == "force_aware" and not planner_path.torque_feasible:
        raise ValueError("force-aware selected path must be torque feasible")

    joint_names = list(result.joint_names)
    joint_path = np.asarray(planner_path.joint_path, dtype=float)
    expected_shape = (len(joint_path), len(joint_names))
    if joint_path.ndim != 2 or joint_path.shape != expected_shape:
        raise ValueError(
            f"{planner_name} joint path must have shape "
            f"(num_waypoints, {len(joint_names)})"
        )
    if len(joint_path) == 0:
        raise ValueError(f"{planner_name} joint path must contain waypoints")
    if not np.all(np.isfinite(joint_path)):
        raise ValueError(f"{planner_name} joint path must contain finite values")

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points = [
        JointTrajectoryPoint(
            positions=joint_positions.tolist(),
            time_from_start=_duration_from_seconds(
                waypoint_interval_s * (waypoint_index + 1)
            ),
        )
        for waypoint_index, joint_positions in enumerate(joint_path)
    ]
    return trajectory


def build_force_aware_joint_trajectory(
    result: Phase1ResultData,
    waypoint_interval_s: float = DEFAULT_WAYPOINT_INTERVAL_S,
) -> JointTrajectory:
    """Return only the torque-feasible selected force-aware path."""

    return build_selected_joint_trajectory(
        result,
        "force_aware",
        waypoint_interval_s,
    )


def build_move_to_first_waypoint_trajectory(
    result: Phase1ResultData,
    move_duration_s: float = DEFAULT_FIRST_WAYPOINT_MOVE_DURATION_S,
    planner_name: str = "force_aware",
) -> JointTrajectory:
    """Return a single-point trajectory that positions at a planned start."""

    full_trajectory = build_selected_joint_trajectory(
        result,
        planner_name,
        waypoint_interval_s=move_duration_s,
    )
    move_trajectory = JointTrajectory()
    move_trajectory.joint_names = full_trajectory.joint_names
    move_trajectory.points = [full_trajectory.points[0]]
    return move_trajectory


def build_force_aware_execution_trajectories(
    result: Phase1ResultData,
    first_waypoint_move_duration_s: float = (
        DEFAULT_FIRST_WAYPOINT_MOVE_DURATION_S
    ),
    waypoint_interval_s: float = DEFAULT_WAYPOINT_INTERVAL_S,
) -> tuple[JointTrajectory, JointTrajectory]:
    """Return the ordered first-positioning move and selected execution path."""

    return build_execution_trajectories(
        result,
        "force_aware",
        first_waypoint_move_duration_s,
        waypoint_interval_s,
    )


def build_execution_trajectories(
    result: Phase1ResultData,
    planner_name: str,
    first_waypoint_move_duration_s: float = (
        DEFAULT_FIRST_WAYPOINT_MOVE_DURATION_S
    ),
    waypoint_interval_s: float = DEFAULT_WAYPOINT_INTERVAL_S,
) -> tuple[JointTrajectory, JointTrajectory]:
    """Return the positioning move and path for an explicitly selected planner."""

    return (
        build_move_to_first_waypoint_trajectory(
            result,
            move_duration_s=first_waypoint_move_duration_s,
            planner_name=planner_name,
        ),
        build_selected_joint_trajectory(
            result,
            planner_name,
            waypoint_interval_s=waypoint_interval_s,
        ),
    )
