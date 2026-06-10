"""Tests for the ros2_control controller configuration."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS_YAML = PACKAGE_ROOT / "config" / "controllers.yaml"
XACRO_PATH = PACKAGE_ROOT / "urdf" / "planar_tool_arm.urdf.xacro"


def load_controller_config() -> dict:
    """Load the controller YAML configuration."""

    with CONTROLLERS_YAML.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def control_joint_interfaces() -> dict[str, tuple[list[str], list[str]]]:
    """Return command and state interfaces from the generated URDF."""

    completed = subprocess.run(
        ["xacro", str(XACRO_PATH)],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = ET.fromstring(completed.stdout)
    control = robot.find("ros2_control")
    assert control is not None
    return {
        joint.attrib["name"]: (
            [
                interface.attrib["name"]
                for interface in joint.findall("command_interface")
            ],
            [
                interface.attrib["name"]
                for interface in joint.findall("state_interface")
            ],
        )
        for joint in control.findall("joint")
    }


def test_controller_manager_declares_required_controllers() -> None:
    config = load_controller_config()
    manager = config["controller_manager"]["ros__parameters"]

    assert manager["update_rate"] == 100
    assert manager["joint_state_broadcaster"]["type"] == (
        "joint_state_broadcaster/JointStateBroadcaster"
    )
    assert manager["force_aware_trajectory_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )


def test_trajectory_controller_matches_mock_hardware_interfaces() -> None:
    config = load_controller_config()
    controller = config["force_aware_trajectory_controller"]["ros__parameters"]

    assert controller["joints"] == ["joint1", "joint2", "joint3"]
    assert controller["command_interfaces"] == ["position"]
    assert controller["state_interfaces"] == ["position"]
    assert controller["allow_partial_joints_goal"] is False
    assert controller["open_loop_control"] is False
    assert control_joint_interfaces() == {
        joint_name: (["position"], ["position"])
        for joint_name in controller["joints"]
    }
