import numpy as np
import pytest

from force_tool_planning.control.force_aware_controller import ForceAwareController


def test_force_aware_controller_tracks_tangential_position_and_velocity() -> None:
    controller = ForceAwareController(
        kp_tangent=2.0,
        kd_tangent=0.5,
        force_gain_mps_per_n=0.1,
        max_normal_correction_mps=1.0,
        force_deadband_n=0.0,
    )

    command = controller.compute_tool_tip_command(
        desired_pos_m=np.array([1.0, 0.0]),
        desired_vel_mps=np.array([0.2, 0.0]),
        current_pos_m=np.array([0.8, 0.4]),
        current_vel_mps=np.array([0.1, -0.2]),
        surface_tangent=np.array([1.0, 0.0]),
        surface_normal=np.array([0.0, 1.0]),
        desired_normal_force_n=5.0,
        measured_normal_force_n=5.0,
    )

    np.testing.assert_allclose(command, np.array([0.65, 0.0]))


def test_positive_force_error_moves_opposite_surface_normal() -> None:
    controller = ForceAwareController(
        kp_tangent=0.0,
        kd_tangent=0.0,
        force_gain_mps_per_n=0.1,
        max_normal_correction_mps=1.0,
        force_deadband_n=0.0,
    )

    command = controller.compute_tool_tip_command(
        desired_pos_m=[0.0, 0.0],
        desired_vel_mps=[0.0, 0.0],
        current_pos_m=[0.0, 0.0],
        current_vel_mps=[0.0, 0.0],
        surface_tangent=[1.0, 0.0],
        surface_normal=[0.0, 1.0],
        desired_normal_force_n=5.0,
        measured_normal_force_n=2.0,
    )

    np.testing.assert_allclose(command, np.array([0.0, -0.3]))


def test_negative_force_error_moves_with_surface_normal() -> None:
    controller = ForceAwareController(
        kp_tangent=0.0,
        kd_tangent=0.0,
        force_gain_mps_per_n=0.1,
        max_normal_correction_mps=1.0,
        force_deadband_n=0.0,
    )

    command = controller.compute_tool_tip_command(
        desired_pos_m=[0.0, 0.0],
        desired_vel_mps=[0.0, 0.0],
        current_pos_m=[0.0, 0.0],
        current_vel_mps=[0.0, 0.0],
        surface_tangent=[1.0, 0.0],
        surface_normal=[0.0, 1.0],
        desired_normal_force_n=2.0,
        measured_normal_force_n=5.0,
    )

    np.testing.assert_allclose(command, np.array([0.0, 0.3]))


def test_force_deadband_removes_small_normal_force_errors() -> None:
    controller = ForceAwareController(
        kp_tangent=0.0,
        kd_tangent=0.0,
        force_gain_mps_per_n=0.1,
        max_normal_correction_mps=1.0,
        force_deadband_n=0.5,
    )

    command = controller.compute_tool_tip_command(
        desired_pos_m=[0.0, 0.0],
        desired_vel_mps=[0.0, 0.0],
        current_pos_m=[0.0, 0.0],
        current_vel_mps=[0.0, 0.0],
        surface_tangent=[1.0, 0.0],
        surface_normal=[0.0, 1.0],
        desired_normal_force_n=5.0,
        measured_normal_force_n=4.7,
    )

    np.testing.assert_allclose(command, np.array([0.0, 0.0]))


def test_normal_force_correction_is_clamped() -> None:
    controller = ForceAwareController(
        kp_tangent=0.0,
        kd_tangent=0.0,
        force_gain_mps_per_n=1.0,
        max_normal_correction_mps=0.2,
        force_deadband_n=0.0,
    )

    command = controller.compute_tool_tip_command(
        desired_pos_m=[0.0, 0.0],
        desired_vel_mps=[0.0, 0.0],
        current_pos_m=[0.0, 0.0],
        current_vel_mps=[0.0, 0.0],
        surface_tangent=[1.0, 0.0],
        surface_normal=[0.0, 2.0],
        desired_normal_force_n=10.0,
        measured_normal_force_n=0.0,
    )

    np.testing.assert_allclose(command, np.array([0.0, -0.2]))


def test_force_aware_controller_validation_rejects_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="kp_tangent"):
        ForceAwareController(
            kp_tangent=-1.0,
            kd_tangent=1.0,
            force_gain_mps_per_n=0.1,
            max_normal_correction_mps=1.0,
            force_deadband_n=0.0,
        )

    controller = ForceAwareController(
        kp_tangent=1.0,
        kd_tangent=1.0,
        force_gain_mps_per_n=0.1,
        max_normal_correction_mps=1.0,
        force_deadband_n=0.0,
    )

    with pytest.raises(ValueError, match="surface_tangent"):
        controller.compute_tool_tip_command(
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0],
            1.0,
            1.0,
        )

    with pytest.raises(ValueError, match="orthogonal"):
        controller.compute_tool_tip_command(
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            1.0,
            1.0,
        )

    with pytest.raises(ValueError, match="measured_normal_force_n"):
        controller.compute_tool_tip_command(
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [1.0, 0.0],
            [0.0, 1.0],
            1.0,
            np.nan,
        )
