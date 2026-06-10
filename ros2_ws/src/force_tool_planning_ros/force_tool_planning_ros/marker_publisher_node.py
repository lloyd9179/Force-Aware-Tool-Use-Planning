"""Publish retained Phase 1 comparison markers for RViz consumers."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from visualization_msgs.msg import MarkerArray

from force_tool_planning_ros.marker_helpers import build_diagnostic_markers
from force_tool_planning_ros.result_adapter import load_phase1_result_data
from force_tool_planning_ros.summary_node import default_config_path

MARKER_TOPIC = "/force_tool_planning/diagnostic_markers"


def marker_qos_profile() -> QoSProfile:
    """Return reliable, transient-local QoS for retained RViz markers."""

    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class MarkerPublisherNode(Node):
    """Publish deterministic comparison markers and retain them for RViz."""

    def __init__(self) -> None:
        super().__init__("force_tool_planning_marker_publisher")
        self.declare_parameter("config_path", str(default_config_path()))
        config_path = Path(
            self.get_parameter("config_path")
            .get_parameter_value()
            .string_value
        )
        result = load_phase1_result_data(config_path)
        self._markers = build_diagnostic_markers(result)
        self._publisher = self.create_publisher(
            MarkerArray,
            MARKER_TOPIC,
            marker_qos_profile(),
        )
        self._publisher.publish(self._markers)
        self.get_logger().info(
            f"Published {len(self._markers.markers)} retained diagnostic "
            f"markers on {MARKER_TOPIC}."
        )


def main(args: list[str] | None = None) -> None:
    """Run the retained diagnostic marker publisher."""

    rclpy.init(args=args)
    node = MarkerPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
