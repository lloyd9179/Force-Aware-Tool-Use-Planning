"""Tests for the position-only ros2_control mock-hardware description."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_ROOT / "urdf" / "planar_tool_arm.urdf.xacro"


def generated_robot() -> ET.Element:
    """Generate and parse the planar arm URDF."""

    completed = subprocess.run(
        ["xacro", str(XACRO_PATH)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(completed.stdout)


def test_mock_hardware_system_uses_generic_position_interface() -> None:
    robot = generated_robot()
    control = robot.find("ros2_control")

    assert control is not None
    assert control.attrib == {
        "name": "PlanarToolArmSystem",
        "type": "system",
    }
    assert control.findtext("hardware/plugin") == (
        "mock_components/GenericSystem"
    )
    assert control.findtext(
        "hardware/param[@name='state_following_offset']"
    ) == "0.0"


def test_mock_hardware_exposes_only_actuated_joint_positions() -> None:
    robot = generated_robot()
    control = robot.find("ros2_control")
    assert control is not None

    control_joints = control.findall("joint")
    assert [joint.attrib["name"] for joint in control_joints] == [
        "joint1",
        "joint2",
        "joint3",
    ]
    for joint in control_joints:
        assert [
            interface.attrib["name"]
            for interface in joint.findall("command_interface")
        ] == ["position"]
        assert [
            interface.attrib["name"]
            for interface in joint.findall("state_interface")
        ] == ["position"]
        assert joint.findtext(
            "state_interface/param[@name='initial_value']"
        ) == "0.0"
