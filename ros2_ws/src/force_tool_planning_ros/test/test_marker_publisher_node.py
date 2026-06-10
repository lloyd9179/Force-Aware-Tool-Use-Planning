"""Tests for diagnostic marker publisher configuration."""

from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy

from force_tool_planning_ros.marker_publisher_node import (
    MARKER_TOPIC,
    marker_qos_profile,
)


def test_marker_publisher_uses_retained_reliable_topic() -> None:
    qos = marker_qos_profile()

    assert MARKER_TOPIC == "/force_tool_planning/diagnostic_markers"
    assert qos.depth == 1
    assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == QoSReliabilityPolicy.RELIABLE
