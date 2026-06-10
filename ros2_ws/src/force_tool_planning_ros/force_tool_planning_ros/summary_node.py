"""Print the deterministic baseline-versus-force-aware Phase 1 comparison."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from force_tool_planning_ros.result_adapter import (
    Phase1ResultData,
    load_phase1_result_data,
)


def format_result_summary(result: Phase1ResultData) -> tuple[str, ...]:
    """Return concise terminal lines describing the selected Phase 1 paths."""

    baseline_feasible = "yes" if result.baseline.torque_feasible else "no"
    force_aware_feasible = (
        "yes" if result.force_aware.torque_feasible else "no"
    )
    return (
        "=== Phase 2 Baseline vs Force-Aware Summary ===",
        f"Frame: {result.frame_id}",
        f"Waypoints: {len(result.tool_path)}",
        f"Desired wrench [Fx, Fy, Mz]: {result.desired_wrench.tolist()}",
        (
            "Baseline: "
            f"grasp={result.baseline.selected_grasp}, "
            f"torque_feasible={baseline_feasible}, "
            f"max_torque_ratio={result.baseline.max_torque_ratio:.6f}"
        ),
        (
            "Baseline violation waypoints: "
            f"{list(result.baseline_violation_waypoint_indices)}"
        ),
        (
            "Force-aware: "
            f"grasp={result.force_aware.selected_grasp}, "
            f"torque_feasible={force_aware_feasible}, "
            f"max_torque_ratio={result.force_aware.max_torque_ratio:.6f}"
        ),
        "Execution selection: force-aware path only",
    )


def default_config_path() -> Path:
    """Return the installed deterministic Phase 1 scenario path."""

    package_share = Path(
        get_package_share_directory("force_tool_planning_ros")
    )
    return package_share / "config" / "demo_planar_arm.yaml"


class ResultSummaryNode(Node):
    """Load the deterministic planner comparison and print its summary once."""

    def __init__(self) -> None:
        super().__init__("baseline_vs_force_aware_summary")
        self.declare_parameter("config_path", str(default_config_path()))
        config_path = Path(
            self.get_parameter("config_path")
            .get_parameter_value()
            .string_value
        )
        result = load_phase1_result_data(config_path)
        for line in format_result_summary(result):
            self.get_logger().info(line)


def main(args: list[str] | None = None) -> None:
    """Run the one-shot baseline-versus-force-aware summary node."""

    rclpy.init(args=args)
    node = ResultSummaryNode()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
