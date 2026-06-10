"""Tests for the ros2_control-only launch configuration."""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROL_LAUNCH = PACKAGE_ROOT / "launch" / "control.launch.py"


def test_control_launch_starts_control_manager_and_required_controllers() -> None:
    launch_source = CONTROL_LAUNCH.read_text(encoding="utf-8")

    assert 'executable="ros2_control_node"' in launch_source
    assert 'executable="robot_state_publisher"' in launch_source
    assert 'arguments=["joint_state_broadcaster"]' in launch_source
    assert 'arguments=["force_aware_trajectory_controller"]' in launch_source
    assert 'remappings=[("~/robot_description", "/robot_description")]' in (
        launch_source
    )


def test_control_launch_does_not_start_display_only_publishers() -> None:
    launch_source = CONTROL_LAUNCH.read_text(encoding="utf-8")

    assert "joint_state_demo_node" not in launch_source
    assert "marker_publisher_node" not in launch_source
    assert "rviz2" not in launch_source
