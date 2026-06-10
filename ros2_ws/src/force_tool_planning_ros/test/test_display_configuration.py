"""Tests for the display launch and RViz planning-diagnostics configuration."""

from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DISPLAY_LAUNCH = PACKAGE_ROOT / "launch" / "display.launch.py"
DISPLAY_RVIZ = PACKAGE_ROOT / "config" / "display.rviz"


def test_display_launch_starts_marker_publisher() -> None:
    launch_source = DISPLAY_LAUNCH.read_text(encoding="utf-8")

    assert 'executable="marker_publisher_node"' in launch_source


def test_rviz_subscribes_to_retained_diagnostic_markers() -> None:
    config = yaml.safe_load(DISPLAY_RVIZ.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    marker_display = next(
        display
        for display in displays
        if display.get("Class") == "rviz_default_plugins/MarkerArray"
    )

    assert marker_display["Enabled"] is True
    assert marker_display["Name"] == "Planning Diagnostics"
    assert marker_display["Topic"] == {
        "Depth": 1,
        "Durability Policy": "Transient Local",
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable",
        "Value": "/force_tool_planning/diagnostic_markers",
    }
    assert marker_display["Namespaces"] == {
        "baseline_ee_path": True,
        "baseline_torque_violations": True,
        "desired_tool_path": True,
        "force_aware_ee_path": True,
    }
