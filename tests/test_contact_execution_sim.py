import numpy as np
import pytest

from force_tool_planning.contact.contact_model import ContactState, PlanarContactModel
from force_tool_planning.contact.surface import Surface2D
from force_tool_planning.control.force_aware_controller import ForceAwareController
from force_tool_planning.control.position_controller import PositionOnlyController
from force_tool_planning.simulation.contact_execution_sim import ContactExecutionSimulator


def _flat_contact_model() -> PlanarContactModel:
    return PlanarContactModel(
        normal_stiffness_n_per_m=100.0,
        normal_damping_n_s_per_m=0.0,
        friction_coefficient=0.4,
        max_penetration_m=0.2,
    )


def test_position_only_simulator_uses_deterministic_kinematic_update() -> None:
    simulator = ContactExecutionSimulator(
        surface=Surface2D("flat", planned_height_m=0.0),
        contact_model=_flat_contact_model(),
        torque_limits_nm=np.array([2.0, 4.0]),
        torque_estimator=lambda _pos, _vel, contact: np.array(
            [contact.normal_force_n, -contact.normal_force_n]
        ),
    )
    controller = PositionOnlyController(kp_task=0.0, kd_task=0.0)

    result = simulator.run(
        controller,
        controller_name="position_only",
        time_s=np.array([0.0, 1.0, 2.0]),
        desired_tool_tip_pos_m=np.array(
            [
                [0.0, -0.01],
                [1.0, -0.01],
                [2.0, -0.01],
            ]
        ),
        desired_tool_tip_vel_mps=np.array(
            [
                [1.0, 0.0],
                [1.0, 0.0],
                [1.0, 0.0],
            ]
        ),
        desired_normal_force_n=1.0,
    )

    np.testing.assert_allclose(
        result.actual_tool_tip_pos_m,
        np.array(
            [
                [0.0, -0.01],
                [1.0, -0.01],
                [2.0, -0.01],
            ]
        ),
    )
    np.testing.assert_allclose(result.normal_force_n, np.array([1.0, 1.0, 1.0]))
    np.testing.assert_allclose(result.torque_ratio, np.array([0.5, 0.5, 0.5]))
    assert result.controller_name == "position_only"


def test_force_aware_simulator_uses_measured_contact_force() -> None:
    simulator = ContactExecutionSimulator(
        surface=Surface2D("flat", planned_height_m=0.0),
        contact_model=_flat_contact_model(),
        torque_limits_nm=np.array([1.0]),
    )
    controller = ForceAwareController(
        kp_tangent=0.0,
        kd_tangent=0.0,
        force_gain_mps_per_n=0.1,
        max_normal_correction_mps=0.2,
        force_deadband_n=0.0,
    )

    result = simulator.run(
        controller,
        controller_name="force_aware",
        time_s=np.array([0.0, 1.0]),
        desired_tool_tip_pos_m=np.array([[0.0, 0.0], [0.0, 0.0]]),
        desired_tool_tip_vel_mps=np.array([[0.0, 0.0], [0.0, 0.0]]),
        desired_normal_force_n=1.0,
    )

    assert result.normal_force_n[0] == pytest.approx(0.0)
    assert result.actual_tool_tip_pos_m[1, 1] == pytest.approx(-0.1)
    assert result.penetration_m[1] == pytest.approx(0.1)
    assert result.normal_force_n[1] == pytest.approx(10.0)


def test_simulator_accepts_time_varying_desired_normal_force() -> None:
    simulator = ContactExecutionSimulator(
        surface=Surface2D("flat", planned_height_m=0.0),
        contact_model=_flat_contact_model(),
        torque_limits_nm=np.array([1.0]),
    )

    result = simulator.run(
        PositionOnlyController(kp_task=0.0, kd_task=0.0),
        controller_name="position_only",
        time_s=np.array([0.0, 0.5]),
        desired_tool_tip_pos_m=np.array([[0.0, -0.01], [0.0, -0.01]]),
        desired_tool_tip_vel_mps=np.zeros((2, 2)),
        desired_normal_force_n=np.array([1.0, 2.0]),
    )

    np.testing.assert_allclose(result.desired_normal_force_n, np.array([1.0, 2.0]))


def test_simulator_validation_rejects_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="torque_limits_nm"):
        ContactExecutionSimulator(
            surface=Surface2D("flat", planned_height_m=0.0),
            contact_model=_flat_contact_model(),
            torque_limits_nm=np.array([0.0]),
        )

    simulator = ContactExecutionSimulator(
        surface=Surface2D("flat", planned_height_m=0.0),
        contact_model=_flat_contact_model(),
        torque_limits_nm=np.array([1.0]),
    )

    with pytest.raises(ValueError, match="time_s"):
        simulator.run(
            PositionOnlyController(kp_task=0.0, kd_task=0.0),
            controller_name="position_only",
            time_s=np.array([0.0, 0.0]),
            desired_tool_tip_pos_m=np.zeros((2, 2)),
            desired_tool_tip_vel_mps=np.zeros((2, 2)),
            desired_normal_force_n=1.0,
        )

    with pytest.raises(ValueError, match="desired_tool_tip_vel_mps"):
        simulator.run(
            PositionOnlyController(kp_task=0.0, kd_task=0.0),
            controller_name="position_only",
            time_s=np.array([0.0, 1.0]),
            desired_tool_tip_pos_m=np.zeros((2, 2)),
            desired_tool_tip_vel_mps=np.zeros((1, 2)),
            desired_normal_force_n=1.0,
        )


def test_simulator_rejects_wrong_torque_estimator_shape() -> None:
    def bad_estimator(
        _pos: np.ndarray,
        _vel: np.ndarray,
        _contact: ContactState,
    ) -> np.ndarray:
        return np.array([0.0, 0.0])

    simulator = ContactExecutionSimulator(
        surface=Surface2D("flat", planned_height_m=0.0),
        contact_model=_flat_contact_model(),
        torque_limits_nm=np.array([1.0]),
        torque_estimator=bad_estimator,
    )

    with pytest.raises(ValueError, match="torque_estimator result"):
        simulator.run(
            PositionOnlyController(kp_task=0.0, kd_task=0.0),
            controller_name="position_only",
            time_s=np.array([0.0]),
            desired_tool_tip_pos_m=np.array([[0.0, -0.01]]),
            desired_tool_tip_vel_mps=np.array([[0.0, 0.0]]),
            desired_normal_force_n=1.0,
        )
