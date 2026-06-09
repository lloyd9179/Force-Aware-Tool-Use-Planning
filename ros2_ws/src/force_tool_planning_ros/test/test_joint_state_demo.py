"""Tests for the deterministic display-only joint-state motion."""

import math

from force_tool_planning_ros.joint_state_demo_node import (
    JOINT_NAMES,
    deterministic_joint_positions,
)


def test_joint_state_names_match_urdf() -> None:
    assert JOINT_NAMES == ("joint1", "joint2", "joint3")


def test_joint_motion_is_repeatable_and_bounded() -> None:
    first = deterministic_joint_positions(2.5)
    second = deterministic_joint_positions(2.5)

    assert first == second
    assert len(first) == 3
    assert all(math.isfinite(position) for position in first)
    assert abs(first[0]) <= 0.55
    assert abs(first[1]) <= 0.70
    assert abs(first[2]) <= 0.55
