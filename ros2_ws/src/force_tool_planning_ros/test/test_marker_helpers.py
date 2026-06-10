"""Tests for deterministic Phase 2 diagnostic marker construction."""

from pathlib import Path

import pytest
from visualization_msgs.msg import Marker

from force_tool_planning_ros.marker_helpers import (
    BASELINE_EE_PATH_ID,
    BASELINE_VIOLATIONS_ID,
    DESIRED_TOOL_PATH_ID,
    FORCE_AWARE_EE_PATH_ID,
    build_diagnostic_markers,
)
from force_tool_planning_ros.result_adapter import load_phase1_result_data

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


@pytest.fixture
def markers() -> list[Marker]:
    result = load_phase1_result_data(DEFAULT_CONFIG)
    return build_diagnostic_markers(result).markers


def test_marker_ids_namespaces_frames_and_types(markers: list[Marker]) -> None:
    assert [(marker.id, marker.ns) for marker in markers] == [
        (DESIRED_TOOL_PATH_ID, "desired_tool_path"),
        (BASELINE_EE_PATH_ID, "baseline_ee_path"),
        (BASELINE_VIOLATIONS_ID, "baseline_torque_violations"),
        (FORCE_AWARE_EE_PATH_ID, "force_aware_ee_path"),
    ]
    assert [marker.header.frame_id for marker in markers] == ["base_link"] * 4
    assert [marker.type for marker in markers] == [
        Marker.LINE_STRIP,
        Marker.LINE_STRIP,
        Marker.SPHERE_LIST,
        Marker.LINE_STRIP,
    ]
    assert all(marker.action == Marker.ADD for marker in markers)


def test_marker_colors_and_point_counts(markers: list[Marker]) -> None:
    assert len(markers) == 4
    assert [len(marker.points) for marker in markers] == [20, 20, 20, 20]
    assert [
        (marker.color.r, marker.color.g, marker.color.b, marker.color.a)
        for marker in markers
    ] == pytest.approx(
        [
            (0.95, 0.95, 0.95, 1.0),
            (0.95, 0.55, 0.10, 1.0),
            (0.95, 0.05, 0.05, 1.0),
            (0.10, 0.80, 0.30, 1.0),
        ]
    )


def test_marker_points_match_selected_paths(markers: list[Marker]) -> None:
    result = load_phase1_result_data(DEFAULT_CONFIG)

    marker_xy = [
        [(point.x, point.y) for point in marker.points]
        for marker in markers
    ]
    assert marker_xy[0] == pytest.approx(
        [(pose[0], pose[1]) for pose in result.tool_path]
    )
    assert marker_xy[1] == pytest.approx(
        [(pose[0], pose[1]) for pose in result.baseline.ee_path]
    )
    assert marker_xy[2] == pytest.approx(
        [
            (
                result.baseline.ee_path[index, 0],
                result.baseline.ee_path[index, 1],
            )
            for index in result.baseline_violation_waypoint_indices
        ]
    )
    assert marker_xy[3] == pytest.approx(
        [(pose[0], pose[1]) for pose in result.force_aware.ee_path]
    )
