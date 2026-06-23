"""RViz marker helpers for Phase 3 live contact execution."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from force_tool_planning.contact.contact_metrics import ContactMetricThresholds
from force_tool_planning.contact.surface import Surface2D
from force_tool_planning.simulation.contact_execution_stepper import (
    ContactExecutionSample,
)

SURFACE_ID = 10
DESIRED_PATH_ID = 11
ACTUAL_PATH_ID = 12
TOOL_TIP_ID = 13
FORCE_NORMAL_ID = 14
STATUS_TEXT_ID = 15


def build_contact_execution_markers(
    *,
    frame_id: str,
    controller_mode: str,
    sample: ContactExecutionSample,
    desired_path_m: np.ndarray,
    actual_path_m: np.ndarray,
    surface: Surface2D,
    thresholds: ContactMetricThresholds,
) -> MarkerArray:
    """Return live Phase 3 contact-execution markers for one sample."""

    return MarkerArray(
        markers=[
            _surface_marker(frame_id, desired_path_m, surface),
            _line_strip(
                marker_id=DESIRED_PATH_ID,
                namespace="phase3_desired_tool_tip_path",
                frame_id=frame_id,
                planar_path=desired_path_m,
                z_m=0.05,
                width_m=0.015,
                color=_color(0.90, 0.90, 0.90, 0.95),
            ),
            _line_strip(
                marker_id=ACTUAL_PATH_ID,
                namespace="phase3_actual_tool_tip_path",
                frame_id=frame_id,
                planar_path=actual_path_m,
                z_m=0.08,
                width_m=0.025,
                color=_controller_color(controller_mode),
            ),
            _tool_tip_marker(frame_id, sample, thresholds),
            _force_normal_marker(frame_id, sample, surface),
            _status_text_marker(frame_id, controller_mode, sample, thresholds),
        ]
    )


def sample_status(
    sample: ContactExecutionSample,
    thresholds: ContactMetricThresholds,
) -> str:
    """Classify the current sample for RViz text and marker color."""

    if sample.torque_ratio > thresholds.torque_failure_ratio:
        return "torque_limit_exceeded"
    if sample.is_excessive_penetration or sample.penetration_m > thresholds.max_penetration_m:
        return "excessive_penetration"
    if (
        not sample.is_in_contact
        or sample.normal_force_n <= thresholds.contact_loss_force_threshold_n
    ):
        return "contact_loss"
    if sample.normal_force_n > thresholds.excessive_force_threshold_n:
        return "excessive_force"
    if sample.torque_ratio >= thresholds.torque_warning_ratio:
        return "torque_warning"
    return "success"


def _surface_marker(
    frame_id: str,
    desired_path_m: np.ndarray,
    surface: Surface2D,
) -> Marker:
    x_min = float(np.min(desired_path_m[:, 0]))
    x_max = float(np.max(desired_path_m[:, 0]))
    x_values = np.linspace(x_min, x_max, 100)
    planar_path = np.column_stack(
        (
            x_values,
            [surface.height(float(x_m)) for x_m in x_values],
        )
    )
    return _line_strip(
        marker_id=SURFACE_ID,
        namespace="phase3_contact_surface",
        frame_id=frame_id,
        planar_path=planar_path,
        z_m=0.02,
        width_m=0.02,
        color=_color(0.35, 0.70, 0.95, 1.0),
    )


def _tool_tip_marker(
    frame_id: str,
    sample: ContactExecutionSample,
    thresholds: ContactMetricThresholds,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "phase3_tool_tip_status"
    marker.id = TOOL_TIP_ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = _point(sample.actual_tool_tip_pos_m, 0.12)
    marker.pose.orientation.w = 1.0
    marker.scale = _scale(0.045, 0.045, 0.045)
    marker.color = _status_color(sample_status(sample, thresholds))
    return marker


def _force_normal_marker(
    frame_id: str,
    sample: ContactExecutionSample,
    surface: Surface2D,
) -> Marker:
    x_m = float(sample.actual_tool_tip_pos_m[0])
    normal = surface.normal(x_m)
    start = np.asarray(sample.actual_tool_tip_pos_m, dtype=float)
    force_scale_m = min(0.25, 0.01 * sample.normal_force_n)
    end = start + force_scale_m * normal

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "phase3_contact_force_normal"
    marker.id = FORCE_NORMAL_ID
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale = _scale(0.018, 0.035, 0.05)
    marker.color = _color(0.95, 0.95, 0.15, 1.0)
    marker.points = [_point(start, 0.14), _point(end, 0.14)]
    return marker


def _status_text_marker(
    frame_id: str,
    controller_mode: str,
    sample: ContactExecutionSample,
    thresholds: ContactMetricThresholds,
) -> Marker:
    status = sample_status(sample, thresholds)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "phase3_contact_status_text"
    marker.id = STATUS_TEXT_ID
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = _point(sample.actual_tool_tip_pos_m, 0.28)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.055
    marker.color = _status_color(status)
    marker.text = (
        f"{controller_mode} | {status}\n"
        f"t={sample.time_s:.2f}s  F={sample.normal_force_n:.2f}N  "
        f"tau={sample.torque_ratio:.2f}"
    )
    return marker


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
    marker.points = [_point(point, z_m) for point in planar_path]
    return marker


def _point(planar_xy_m: np.ndarray, z_m: float) -> Point:
    point = Point()
    point.x = float(planar_xy_m[0])
    point.y = float(planar_xy_m[1])
    point.z = float(z_m)
    return point


def _scale(x_m: float, y_m: float, z_m: float) -> Vector3:
    scale = Vector3()
    scale.x = x_m
    scale.y = y_m
    scale.z = z_m
    return scale


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


def _controller_color(controller_mode: str) -> ColorRGBA:
    if controller_mode == "position_only":
        return _color(0.10, 0.45, 0.85, 1.0)
    return _color(1.0, 0.45, 0.05, 1.0)


def _status_color(status: str) -> ColorRGBA:
    if status == "success":
        return _color(0.10, 0.80, 0.30, 1.0)
    if status == "torque_warning":
        return _color(1.0, 0.70, 0.05, 1.0)
    if status == "contact_loss":
        return _color(0.95, 0.15, 0.15, 1.0)
    if status == "excessive_penetration":
        return _color(0.85, 0.10, 0.95, 1.0)
    if status == "excessive_force":
        return _color(1.0, 0.35, 0.05, 1.0)
    return _color(0.95, 0.05, 0.05, 1.0)
