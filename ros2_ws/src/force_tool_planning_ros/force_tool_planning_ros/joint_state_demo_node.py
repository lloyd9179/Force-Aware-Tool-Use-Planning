"""Publish deterministic joint states for the display-only Phase 2 demo."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ("joint1", "joint2", "joint3")


def deterministic_joint_positions(elapsed_sec: float) -> tuple[float, float, float]:
    """Return smooth, repeatable joint positions in radians."""

    phase = 0.45 * elapsed_sec
    return (
        0.55 * math.sin(phase),
        0.70 * math.sin(phase + 0.8),
        0.55 * math.sin(phase + 1.6),
    )


class JointStateDemoNode(Node):
    """Publish a slow deterministic motion for URDF and RViz validation."""

    def __init__(self) -> None:
        super().__init__("joint_state_demo")
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(0.05, self._publish_joint_state)
        self.get_logger().info("Publishing deterministic display joint states.")

    def _publish_joint_state(self) -> None:
        elapsed_sec = (
            self.get_clock().now() - self._start_time
        ).nanoseconds / 1_000_000_000.0

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(deterministic_joint_positions(elapsed_sec))
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the deterministic joint-state display node."""

    rclpy.init(args=args)
    node = JointStateDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
