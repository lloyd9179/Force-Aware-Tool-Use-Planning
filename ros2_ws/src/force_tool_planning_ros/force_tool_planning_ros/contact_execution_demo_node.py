"""Timer-driven ROS2 wrapper for Phase 3 live contact execution."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from visualization_msgs.msg import MarkerArray

from force_tool_planning.analysis.compare_execution import (
    Phase1ReferenceTorqueEstimator,
    build_contact_execution_stepper,
    build_metric_thresholds,
    load_phase3_config,
)
from force_tool_planning.simulation.contact_execution_stepper import (
    ContactExecutionSample,
)
from force_tool_planning_ros.contact_markers import (
    build_contact_execution_markers,
    sample_status,
)

CONTACT_MARKER_TOPIC = "/force_tool_planning/contact_execution_markers"
CONTACT_STATUS_TOPIC = "/force_tool_planning/contact_execution_status"
CONTACT_NORMAL_FORCE_TOPIC = "/force_tool_planning/contact_normal_force_n"
CONTACT_TORQUE_RATIO_TOPIC = "/force_tool_planning/contact_torque_ratio"
JOINT_NAMES = ("joint1", "joint2", "joint3")


def default_phase3_config_path() -> Path:
    """Return the installed default Phase 3 config path for ROS2 launches."""

    return (
        Path(get_package_share_directory("force_tool_planning_ros"))
        / "config"
        / "phase3_contact_execution.yaml"
    )


def contact_marker_qos_profile() -> QoSProfile:
    """Return reliable transient-local QoS for RViz marker consumers."""

    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Phase3ContactExecutionDemoNode(Node):
    """Advance Phase 3 contact execution from a timer and publish RViz state."""

    def __init__(self) -> None:
        super().__init__("phase3_contact_execution_demo")
        self.declare_parameter("controller_mode", "force_aware")
        self.declare_parameter("config_path", str(default_phase3_config_path()))
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("loop", True)

        self._controller_mode = self._controller_mode_parameter()
        self._config_path = Path(
            self.get_parameter("config_path")
            .get_parameter_value()
            .string_value
        )
        self._frame_id = (
            self.get_parameter("frame_id")
            .get_parameter_value()
            .string_value
        )
        publish_period_s = (
            self.get_parameter("publish_period_s")
            .get_parameter_value()
            .double_value
        )
        self._loop = (
            self.get_parameter("loop")
            .get_parameter_value()
            .bool_value
        )
        if publish_period_s <= 0.0:
            raise ValueError("publish_period_s must be positive")

        self._config = load_phase3_config(self._config_path)
        self._thresholds = build_metric_thresholds(self._config)
        self._stepper = build_contact_execution_stepper(
            self._config,
            self._config_path,
            controller_mode=self._controller_mode,
        )
        self._actual_trace_m: list[np.ndarray] = []

        self._marker_publisher = self.create_publisher(
            MarkerArray,
            CONTACT_MARKER_TOPIC,
            contact_marker_qos_profile(),
        )
        self._status_publisher = self.create_publisher(
            String,
            CONTACT_STATUS_TOPIC,
            10,
        )
        self._normal_force_publisher = self.create_publisher(
            Float64,
            CONTACT_NORMAL_FORCE_TOPIC,
            10,
        )
        self._torque_ratio_publisher = self.create_publisher(
            Float64,
            CONTACT_TORQUE_RATIO_TOPIC,
            10,
        )
        self._joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        self._timer = self.create_timer(publish_period_s, self._on_timer)
        self.get_logger().info(
            "Running Phase 3 contact execution live wrapper with "
            f"controller_mode={self._controller_mode}."
        )

    def _controller_mode_parameter(self) -> str:
        controller_mode = (
            self.get_parameter("controller_mode")
            .get_parameter_value()
            .string_value
        )
        if controller_mode not in ("position_only", "force_aware"):
            raise ValueError(
                "controller_mode must be 'position_only' or 'force_aware'"
            )
        return controller_mode

    def _on_timer(self) -> None:
        sample = self._stepper.step()
        if sample is None:
            if not self._loop:
                return
            self._stepper.reset()
            self._actual_trace_m = []
            sample = self._stepper.step()
            if sample is None:
                return

        self._actual_trace_m.append(sample.actual_tool_tip_pos_m.copy())
        self._publish_sample(sample)

    def _publish_sample(self, sample: ContactExecutionSample) -> None:
        actual_path_m = np.asarray(self._actual_trace_m, dtype=float)
        markers = build_contact_execution_markers(
            frame_id=self._frame_id,
            controller_mode=self._controller_mode,
            sample=sample,
            desired_path_m=self._stepper.desired_tool_tip_pos_m,
            actual_path_m=actual_path_m,
            surface=self._stepper.simulator.surface,
            thresholds=self._thresholds,
        )
        stamp = self.get_clock().now().to_msg()
        for marker in markers.markers:
            marker.header.stamp = stamp
        self._marker_publisher.publish(markers)

        status = sample_status(sample, self._thresholds)
        status_message = String()
        status_message.data = (
            f"controller={self._controller_mode} status={status} "
            f"sample={sample.sample_index}/{self._stepper.sample_count - 1} "
            f"time_s={sample.time_s:.3f} "
            f"normal_force_n={sample.normal_force_n:.3f} "
            f"penetration_m={sample.penetration_m:.5f} "
            f"torque_ratio={sample.torque_ratio:.3f}"
        )
        self._status_publisher.publish(status_message)

        normal_force_message = Float64()
        normal_force_message.data = sample.normal_force_n
        self._normal_force_publisher.publish(normal_force_message)

        torque_ratio_message = Float64()
        torque_ratio_message.data = sample.torque_ratio
        self._torque_ratio_publisher.publish(torque_ratio_message)
        self._publish_joint_state(stamp)

    def _publish_joint_state(self, stamp: object) -> None:
        torque_estimator = self._stepper.simulator.torque_estimator
        if not isinstance(torque_estimator, Phase1ReferenceTorqueEstimator):
            return
        joint_positions = torque_estimator.current_joint_positions_rad
        if joint_positions is None:
            return

        message = JointState()
        message.header.stamp = stamp
        message.name = list(JOINT_NAMES)
        message.position = [float(value) for value in joint_positions]
        self._joint_state_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the Phase 3 live contact execution ROS2 wrapper."""

    rclpy.init(args=args)
    node = Phase3ContactExecutionDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
