"""Tests for live Phase 3 contact execution stepping."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from force_tool_planning.analysis.compare_execution import (
    DEFAULT_PHASE3_CONFIG,
    build_contact_execution_simulator,
    build_controllers,
    build_desired_contact_trajectory,
    load_phase3_config,
)
from force_tool_planning.simulation.contact_execution_stepper import (
    ContactExecutionStepper,
)


def test_live_stepper_matches_batch_simulator_for_force_aware() -> None:
    config = load_phase3_config(DEFAULT_PHASE3_CONFIG)
    trajectory = build_desired_contact_trajectory(config)
    _position_controller, force_controller = build_controllers(config)
    simulator = build_contact_execution_simulator(config, DEFAULT_PHASE3_CONFIG)

    batch_result = simulator.run(
        force_controller,
        controller_name="force_aware",
        time_s=trajectory.time_s,
        desired_tool_tip_pos_m=trajectory.desired_tool_tip_pos_m,
        desired_tool_tip_vel_mps=trajectory.desired_tool_tip_vel_mps,
        desired_normal_force_n=trajectory.desired_normal_force_n,
        initial_tool_tip_pos_m=trajectory.initial_tool_tip_pos_m,
    )
    stepper = ContactExecutionStepper(
        simulator,
        force_controller,
        controller_name="force_aware",
        time_s=trajectory.time_s,
        desired_tool_tip_pos_m=trajectory.desired_tool_tip_pos_m,
        desired_tool_tip_vel_mps=trajectory.desired_tool_tip_vel_mps,
        desired_normal_force_n=trajectory.desired_normal_force_n,
        initial_tool_tip_pos_m=trajectory.initial_tool_tip_pos_m,
    )

    samples = []
    while not stepper.is_done:
        samples.append(stepper.step())

    assert len(samples) == batch_result.sample_count
    assert samples[0] is not None
    assert samples[-1] is not None
    assert samples[0].sample_index == 0
    assert samples[-1].is_last_sample is True

    live_result = stepper.result()
    np.testing.assert_allclose(
        live_result.actual_tool_tip_pos_m,
        batch_result.actual_tool_tip_pos_m,
    )
    np.testing.assert_allclose(live_result.normal_force_n, batch_result.normal_force_n)
    np.testing.assert_allclose(live_result.penetration_m, batch_result.penetration_m)
    np.testing.assert_array_equal(live_result.is_in_contact, batch_result.is_in_contact)
    np.testing.assert_allclose(
        live_result.joint_torque_nm,
        batch_result.joint_torque_nm,
    )
    np.testing.assert_allclose(live_result.torque_ratio, batch_result.torque_ratio)


def test_live_stepper_requires_completion_before_result() -> None:
    config = load_phase3_config(DEFAULT_PHASE3_CONFIG)
    trajectory = build_desired_contact_trajectory(config)
    position_controller, _force_controller = build_controllers(config)
    simulator = build_contact_execution_simulator(config, DEFAULT_PHASE3_CONFIG)
    stepper = ContactExecutionStepper(
        simulator,
        position_controller,
        controller_name="position_only",
        time_s=trajectory.time_s,
        desired_tool_tip_pos_m=trajectory.desired_tool_tip_pos_m,
        desired_tool_tip_vel_mps=trajectory.desired_tool_tip_vel_mps,
        desired_normal_force_n=trajectory.desired_normal_force_n,
        initial_tool_tip_pos_m=trajectory.initial_tool_tip_pos_m,
    )

    with pytest.raises(RuntimeError, match="before all samples"):
        stepper.result()

    assert stepper.step() is not None
    assert stepper.current_index == 1


def test_default_phase3_config_path_exists() -> None:
    assert Path(DEFAULT_PHASE3_CONFIG).exists()
