import inspect

import numpy as np
import pytest

from force_tool_planning.control.position_controller import PositionOnlyController


def test_position_only_controller_tracks_position_and_velocity_errors() -> None:
    controller = PositionOnlyController(kp_task=2.0, kd_task=0.5)

    command = controller.compute_tool_tip_command(
        desired_pos_m=np.array([1.0, 0.5]),
        desired_vel_mps=np.array([0.2, 0.0]),
        current_pos_m=np.array([0.8, 0.7]),
        current_vel_mps=np.array([0.1, -0.1]),
    )

    np.testing.assert_allclose(command, np.array([0.65, -0.35]))


def test_position_only_controller_returns_desired_velocity_at_zero_error() -> None:
    controller = PositionOnlyController(kp_task=4.0, kd_task=1.0)

    command = controller.compute_tool_tip_command(
        desired_pos_m=[0.2, -0.1],
        desired_vel_mps=[0.15, 0.0],
        current_pos_m=[0.2, -0.1],
        current_vel_mps=[0.15, 0.0],
    )

    np.testing.assert_allclose(command, np.array([0.15, 0.0]))


def test_position_only_controller_does_not_accept_force_feedback() -> None:
    signature = inspect.signature(PositionOnlyController.compute_tool_tip_command)

    assert "measured_normal_force" not in signature.parameters
    assert "desired_normal_force" not in signature.parameters


def test_position_only_controller_validation_rejects_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="kp_task"):
        PositionOnlyController(kp_task=-1.0, kd_task=1.0)

    with pytest.raises(ValueError, match="kd_task"):
        PositionOnlyController(kp_task=1.0, kd_task=np.inf)

    controller = PositionOnlyController(kp_task=1.0, kd_task=1.0)

    with pytest.raises(ValueError, match="desired_pos_m"):
        controller.compute_tool_tip_command([0.0, 0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0])

    with pytest.raises(ValueError, match="current_vel_mps"):
        controller.compute_tool_tip_command([0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [np.nan, 0.0])
