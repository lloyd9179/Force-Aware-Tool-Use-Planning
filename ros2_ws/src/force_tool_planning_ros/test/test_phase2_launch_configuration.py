"""Tests for the complete force-aware and baseline comparison demos."""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEMO_HELPER = PACKAGE_ROOT / "force_tool_planning_ros" / "demo_launch.py"
FORCE_AWARE_LAUNCH = PACKAGE_ROOT / "launch" / "phase2.launch.py"
BASELINE_LAUNCH = PACKAGE_ROOT / "launch" / "baseline_demo.launch.py"


def test_shared_demo_helper_starts_and_orders_complete_demo_components() -> None:
    launch_source = DEMO_HELPER.read_text(encoding="utf-8")

    for executable in (
        "robot_state_publisher",
        "ros2_control_node",
        "result_summary_node",
        "marker_publisher_node",
        "rviz2",
        "trajectory_sender_node",
    ):
        assert f'executable="{executable}"' in launch_source
    assert 'arguments=["joint_state_broadcaster"]' in launch_source
    assert 'arguments=["force_aware_trajectory_controller"]' in launch_source
    assert "target_action=trajectory_controller_spawner" in launch_source
    assert "on_exit=[trajectory_sender]" in launch_source
    assert "joint_state_demo_node" not in launch_source


def test_force_aware_demo_runs_selected_path_twice() -> None:
    launch_source = FORCE_AWARE_LAUNCH.read_text(encoding="utf-8")

    assert 'planner_name="force_aware"' in launch_source
    assert "repeat_count=2" in launch_source
    assert "ee_to_tool_x=0.20" in launch_source
    assert "torque feasible" in launch_source


def test_baseline_demo_is_explicitly_labeled_and_runs_twice() -> None:
    launch_source = BASELINE_LAUNCH.read_text(encoding="utf-8")

    assert 'planner_name="baseline"' in launch_source
    assert "repeat_count=2" in launch_source
    assert "ee_to_tool_yaw=0.6" in launch_source
    assert "torque-infeasible visualization only" in launch_source
