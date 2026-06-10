"""Send one explicitly selected planner path to the ros2_control action server."""

from __future__ import annotations

from functools import partial
from pathlib import Path

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from force_tool_planning_ros.result_adapter import load_phase1_result_data
from force_tool_planning_ros.summary_node import default_config_path
from force_tool_planning_ros.trajectory_helpers import (
    DEFAULT_FIRST_WAYPOINT_MOVE_DURATION_S,
    DEFAULT_WAYPOINT_INTERVAL_S,
    PLANNER_NAMES,
    build_execution_trajectories,
    selected_planner_path,
)

ACTION_NAME = (
    "/force_aware_trajectory_controller/follow_joint_trajectory"
)
ACTION_SERVER_TIMEOUT_S = 10.0


class TrajectorySenderNode(Node):
    """Position and execute one selected planner path a fixed number of times."""

    def __init__(self) -> None:
        super().__init__("planner_trajectory_sender")
        self.declare_parameter("config_path", str(default_config_path()))
        self.declare_parameter(
            "first_waypoint_move_duration_s",
            DEFAULT_FIRST_WAYPOINT_MOVE_DURATION_S,
        )
        self.declare_parameter(
            "waypoint_interval_s",
            DEFAULT_WAYPOINT_INTERVAL_S,
        )
        self.declare_parameter("planner_name", "force_aware")
        self.declare_parameter("repeat_count", 1)

        config_path = Path(
            self.get_parameter("config_path")
            .get_parameter_value()
            .string_value
        )
        first_move_duration_s = (
            self.get_parameter("first_waypoint_move_duration_s")
            .get_parameter_value()
            .double_value
        )
        waypoint_interval_s = (
            self.get_parameter("waypoint_interval_s")
            .get_parameter_value()
            .double_value
        )
        self._planner_name = (
            self.get_parameter("planner_name")
            .get_parameter_value()
            .string_value
        )
        self._repeat_count = (
            self.get_parameter("repeat_count")
            .get_parameter_value()
            .integer_value
        )
        if self._planner_name not in PLANNER_NAMES:
            raise ValueError(
                f"planner_name must be one of {PLANNER_NAMES}: "
                f"{self._planner_name}"
            )
        if self._repeat_count <= 0:
            raise ValueError("repeat_count must be positive")

        result = load_phase1_result_data(config_path)
        planner_path = selected_planner_path(result, self._planner_name)
        if not planner_path.torque_feasible:
            self.get_logger().warning(
                "Executing the torque-infeasible baseline path for visual "
                "comparison only."
            )
        (
            self._first_move,
            self._selected_trajectory,
        ) = build_execution_trajectories(
            result,
            self._planner_name,
            first_waypoint_move_duration_s=first_move_duration_s,
            waypoint_interval_s=waypoint_interval_s,
        )
        self._completed_runs = 0
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )
        self._start_timer = self.create_timer(0.1, self._start_execution)

    def _start_execution(self) -> None:
        if self._start_timer is not None:
            self.destroy_timer(self._start_timer)
            self._start_timer = None
        if not self._action_client.wait_for_server(
            timeout_sec=ACTION_SERVER_TIMEOUT_S
        ):
            self._fail(f"Action server unavailable: {ACTION_NAME}")
            return

        self.get_logger().info(
            f"Sending controlled move to the first {self._planner_name} "
            f"waypoint for run {self._completed_runs + 1}/"
            f"{self._repeat_count}."
        )
        self._send_trajectory(
            self._first_move,
            label="first-waypoint move",
            on_success=self._send_selected_path,
        )

    def _send_selected_path(self) -> None:
        self.get_logger().info(
            f"First waypoint reached; sending the {self._planner_name} "
            "joint path."
        )
        self._send_trajectory(
            self._selected_trajectory,
            label=f"{self._planner_name} path",
            on_success=self._complete_run,
        )

    def _complete_run(self) -> None:
        self._completed_runs += 1
        self.get_logger().info(
            f"Completed {self._planner_name} run "
            f"{self._completed_runs}/{self._repeat_count}."
        )
        if self._completed_runs < self._repeat_count:
            self.get_logger().info(
                "The full motion has stopped; starting the next run."
            )
            self._start_execution()
            return
        self._finish_successfully()

    def _send_trajectory(
        self,
        trajectory: JointTrajectory,
        *,
        label: str,
        on_success,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(
            partial(
                self._goal_response_callback,
                label=label,
                on_success=on_success,
            )
        )

    def _goal_response_callback(self, future, *, label: str, on_success) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._fail(f"Controller rejected {label}.")
            return

        self.get_logger().info(f"Controller accepted {label}.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._result_callback,
                label=label,
                on_success=on_success,
            )
        )

    def _result_callback(self, future, *, label: str, on_success) -> None:
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._fail(
                f"{label} failed with error code {result.error_code}: "
                f"{result.error_string}"
            )
            return

        self.get_logger().info(f"Completed {label}.")
        on_success()

    def _finish_successfully(self) -> None:
        self.get_logger().info(
            f"Completed {self._repeat_count} {self._planner_name} "
            "trajectory runs; stopping sender."
        )
        if rclpy.ok():
            rclpy.shutdown()

    def _fail(self, message: str) -> None:
        self.get_logger().error(message)
        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    """Run the one-shot force-aware trajectory sender."""

    rclpy.init(args=args)
    node = TrajectorySenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
