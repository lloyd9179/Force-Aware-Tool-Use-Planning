"""Tests for the Phase 3 ROS2 live contact execution wrapper."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml
from visualization_msgs.msg import Marker

from force_tool_planning.contact.contact_metrics import ContactMetricThresholds
from force_tool_planning.contact.surface import Surface2D
from force_tool_planning.simulation.contact_execution_stepper import (
    ContactExecutionSample,
)
from force_tool_planning_ros.contact_markers import (
    ACTUAL_PATH_ID,
    DESIRED_PATH_ID,
    FORCE_NORMAL_ID,
    STATUS_TEXT_ID,
    SURFACE_ID,
    TOOL_TIP_ID,
    build_contact_execution_markers,
    sample_status,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "phase3_contact_execution.launch.py"
RVIZ_FILE = PACKAGE_ROOT / "config" / "phase3_contact_execution.rviz"
SETUP_FILE = PACKAGE_ROOT / "setup.py"
NODE_FILE = PACKAGE_ROOT / "force_tool_planning_ros" / "contact_execution_demo_node.py"


def _thresholds() -> ContactMetricThresholds:
    return ContactMetricThresholds(
        contact_loss_force_threshold_n=0.2,
        excessive_force_threshold_n=12.0,
        contact_loss_fraction_threshold=0.2,
        torque_warning_ratio=0.9,
        torque_failure_ratio=1.0,
        max_penetration_m=0.05,
    )


def _sample(**overrides: object) -> ContactExecutionSample:
    values = {
        "sample_index": 3,
        "time_s": 0.06,
        "desired_tool_tip_pos_m": np.array([0.0, -0.0125]),
        "actual_tool_tip_pos_m": np.array([0.0, -0.0120]),
        "normal_force_n": 5.0,
        "desired_normal_force_n": 5.0,
        "penetration_m": 0.0120,
        "is_in_contact": True,
        "is_excessive_penetration": False,
        "surface_height_m": 0.0,
        "joint_torque_nm": np.array([1.0, 2.0, 3.0]),
        "torque_ratio": 0.4,
        "is_last_sample": False,
    }
    values.update(overrides)
    return ContactExecutionSample(**values)


def test_contact_marker_ids_namespaces_and_types() -> None:
    markers = build_contact_execution_markers(
        frame_id="base_link",
        controller_mode="force_aware",
        sample=_sample(),
        desired_path_m=np.array([[-0.1, -0.0125], [0.1, -0.0125]]),
        actual_path_m=np.array([[-0.1, -0.0120], [0.0, -0.0120]]),
        surface=Surface2D(surface_type="flat", planned_height_m=0.0),
        thresholds=_thresholds(),
    ).markers

    assert [(marker.id, marker.ns) for marker in markers] == [
        (SURFACE_ID, "phase3_contact_surface"),
        (DESIRED_PATH_ID, "phase3_desired_tool_tip_path"),
        (ACTUAL_PATH_ID, "phase3_actual_tool_tip_path"),
        (TOOL_TIP_ID, "phase3_tool_tip_status"),
        (FORCE_NORMAL_ID, "phase3_contact_force_normal"),
        (STATUS_TEXT_ID, "phase3_contact_status_text"),
    ]
    assert [marker.type for marker in markers] == [
        Marker.LINE_STRIP,
        Marker.LINE_STRIP,
        Marker.LINE_STRIP,
        Marker.SPHERE,
        Marker.ARROW,
        Marker.TEXT_VIEW_FACING,
    ]
    assert [marker.header.frame_id for marker in markers] == ["base_link"] * 6


@pytest.mark.parametrize(
    ("sample", "expected_status"),
    [
        (_sample(), "success"),
        (_sample(is_in_contact=False, normal_force_n=0.0), "contact_loss"),
        (_sample(penetration_m=0.06), "excessive_penetration"),
        (_sample(normal_force_n=13.0), "excessive_force"),
        (_sample(torque_ratio=0.95), "torque_warning"),
        (_sample(torque_ratio=1.2), "torque_limit_exceeded"),
    ],
)
def test_sample_status_distinguishes_contact_failure_modes(
    sample: ContactExecutionSample,
    expected_status: str,
) -> None:
    assert sample_status(sample, _thresholds()) == expected_status


def test_phase3_launch_starts_live_node_robot_model_and_rviz() -> None:
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'executable="robot_state_publisher"' in launch_source
    assert 'executable="contact_execution_demo_node"' in launch_source
    assert 'executable="rviz2"' in launch_source
    assert 'DeclareLaunchArgument(\n                "controller_mode"' in launch_source
    assert "phase3_contact_execution.yaml" in launch_source
    assert "phase3_contact_execution.rviz" in launch_source


def test_phase3_rviz_subscribes_to_live_contact_markers() -> None:
    config = yaml.safe_load(RVIZ_FILE.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    marker_display = next(
        display
        for display in displays
        if display.get("Name") == "Phase 3 Contact Execution"
    )

    assert marker_display["Enabled"] is True
    assert marker_display["Topic"] == {
        "Depth": 1,
        "Durability Policy": "Transient Local",
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable",
        "Value": "/force_tool_planning/contact_execution_markers",
    }
    assert marker_display["Namespaces"] == {
        "phase3_actual_tool_tip_path": True,
        "phase3_contact_force_normal": True,
        "phase3_contact_status_text": True,
        "phase3_contact_surface": True,
        "phase3_desired_tool_tip_path": True,
        "phase3_tool_tip_status": True,
    }


def test_phase3_rviz_view_focuses_on_contact_strip() -> None:
    config = yaml.safe_load(RVIZ_FILE.read_text(encoding="utf-8"))
    view = config["Visualization Manager"]["Views"]["Current"]

    assert view["Class"] == "rviz_default_plugins/Orbit"
    assert view["Target Frame"] == "base_link"
    assert view["Distance"] == pytest.approx(2.2)
    assert view["Focal Point"] == {
        "X": 1.45,
        "Y": 0.6,
        "Z": 0.0,
    }


def test_phase3_node_and_setup_reuse_python_core() -> None:
    node_source = NODE_FILE.read_text(encoding="utf-8")
    setup_source = SETUP_FILE.read_text(encoding="utf-8")

    assert "build_contact_execution_stepper" in node_source
    assert "build_metric_thresholds" in node_source
    assert "contact_execution_demo_node =" in setup_source
    assert "phase3_contact_execution.yaml" in setup_source
