"""Construct deterministic RViz diagnostics from adapted Phase 1 results."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from force_tool_planning_ros.result_adapter import Phase1ResultData

DESIRED_TOOL_PATH_ID = 0
BASELINE_EE_PATH_ID = 1
BASELINE_VIOLATIONS_ID = 2
FORCE_AWARE_EE_PATH_ID = 3


def _color(
    red: float,
    green: float,
    blue: float,
    alpha: float = 1.0,
) -> ColorRGBA:
    color = ColorRGBA()
    color.r = red
    color.g = green
    color.b = blue
    color.a = alpha
    return color


def _points(planar_path: np.ndarray, z_m: float) -> list[Point]:
    points: list[Point] = []
    for x_m, y_m, _theta_rad in planar_path:
        point = Point()
        point.x = float(x_m)
        point.y = float(y_m)
        point.z = z_m
        points.append(point)
    return points


def _point(planar_pose: np.ndarray, z_m: float) -> Point:
    return _points(np.asarray([planar_pose], dtype=float), z_m)[0]


def _line_strip(
    *,
    marker_id: int,
    namespace: str,
    frame_id: str,
    planar_path: np.ndarray,
    z_m: float,
    width_m: float,
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width_m
    marker.color = color
    marker.points = _points(planar_path, z_m)
    return marker


def build_diagnostic_markers(result: Phase1ResultData) -> MarkerArray:
    """Return desired, baseline, violation, and force-aware RViz markers."""

    desired_path = _line_strip(
        marker_id=DESIRED_TOOL_PATH_ID,
        namespace="desired_tool_path",
        frame_id=result.frame_id,
        planar_path=result.tool_path,
        z_m=0.10,
        width_m=0.025,
        color=_color(0.95, 0.95, 0.95),
    )
    baseline_path = _line_strip(
        marker_id=BASELINE_EE_PATH_ID,
        namespace="baseline_ee_path",
        frame_id=result.frame_id,
        planar_path=result.baseline.ee_path,
        z_m=0.14,
        width_m=0.035,
        color=_color(0.95, 0.55, 0.10),
    )

    violations = Marker()
    violations.header.frame_id = result.frame_id
    violations.ns = "baseline_torque_violations"
    violations.id = BASELINE_VIOLATIONS_ID
    violations.type = Marker.SPHERE_LIST
    violations.action = Marker.ADD
    violations.pose.orientation.w = 1.0
    violations.scale.x = 0.09
    violations.scale.y = 0.09
    violations.scale.z = 0.09
    violations.color = _color(0.95, 0.05, 0.05)
    violations.points = [
        _point(result.baseline.ee_path[index], 0.18)
        for index in result.baseline_violation_waypoint_indices
    ]

    force_aware_path = _line_strip(
        marker_id=FORCE_AWARE_EE_PATH_ID,
        namespace="force_aware_ee_path",
        frame_id=result.frame_id,
        planar_path=result.force_aware.ee_path,
        z_m=0.22,
        width_m=0.045,
        color=_color(0.10, 0.80, 0.30),
    )
    return MarkerArray(
        markers=[
            desired_path,
            baseline_path,
            violations,
            force_aware_path,
        ]
    )
