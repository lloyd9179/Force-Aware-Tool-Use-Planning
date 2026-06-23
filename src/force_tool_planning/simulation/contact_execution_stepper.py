"""Live stepping wrapper for deterministic Phase 3 contact execution."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from force_tool_planning.control.force_aware_controller import ForceAwareController
from force_tool_planning.control.position_controller import PositionOnlyController
from force_tool_planning.simulation.contact_execution_sim import ContactExecutionSimulator
from force_tool_planning.simulation.execution_result import ContactExecutionResult


@dataclass(frozen=True)
class ContactExecutionSample:
    """One deterministic live contact-execution sample."""

    sample_index: int
    time_s: float
    desired_tool_tip_pos_m: np.ndarray
    actual_tool_tip_pos_m: np.ndarray
    normal_force_n: float
    desired_normal_force_n: float
    penetration_m: float
    is_in_contact: bool
    is_excessive_penetration: bool
    surface_height_m: float
    joint_torque_nm: np.ndarray
    torque_ratio: float
    is_last_sample: bool


class ContactExecutionStepper:
    """Advance the same deterministic simulator one sample per call.

    This class is intentionally ROS-independent. ROS2 wrappers can own a
    stepper and call :meth:`step` from a timer while the contact, controller,
    and torque computations stay in the pure-Python package.
    """

    def __init__(
        self,
        simulator: ContactExecutionSimulator,
        controller: PositionOnlyController | ForceAwareController,
        *,
        controller_name: str,
        time_s: ArrayLike,
        desired_tool_tip_pos_m: ArrayLike,
        desired_tool_tip_vel_mps: ArrayLike,
        desired_normal_force_n: ArrayLike | float,
        initial_tool_tip_pos_m: ArrayLike | None = None,
    ) -> None:
        self.simulator = simulator
        self.controller = controller
        self.controller_name = str(controller_name).strip()
        if not self.controller_name:
            raise ValueError("controller_name must be non-empty")

        self.time_s = simulator._as_1d_float(time_s, "time_s")
        if self.time_s.size == 0:
            raise ValueError("time_s must contain at least one sample")
        if self.time_s.size > 1 and np.any(np.diff(self.time_s) <= 0.0):
            raise ValueError("time_s must be strictly increasing")

        self.desired_tool_tip_pos_m = simulator._as_matrix(
            desired_tool_tip_pos_m,
            "desired_tool_tip_pos_m",
            self.time_s.size,
            column_count=2,
        )
        self.desired_tool_tip_vel_mps = simulator._as_matrix(
            desired_tool_tip_vel_mps,
            "desired_tool_tip_vel_mps",
            self.time_s.size,
            column_count=2,
        )
        self.desired_normal_force_n = simulator._as_1d_float_or_scalar(
            desired_normal_force_n,
            "desired_normal_force_n",
            self.time_s.size,
        )
        self.initial_tool_tip_pos_m = (
            simulator._as_vector2(initial_tool_tip_pos_m, "initial_tool_tip_pos_m")
            if initial_tool_tip_pos_m is not None
            else self.desired_tool_tip_pos_m[0].copy()
        )
        self.reset()

    @property
    def sample_count(self) -> int:
        """Return the number of samples in the configured trajectory."""

        return int(self.time_s.size)

    @property
    def current_index(self) -> int:
        """Return the next sample index that will be computed."""

        return self._index

    @property
    def is_done(self) -> bool:
        """Return whether all samples have been generated."""

        return self._index >= self.sample_count

    def reset(self) -> None:
        """Reset the live rollout to the initial tool-tip state."""

        torque_estimator = self.simulator.torque_estimator
        if hasattr(torque_estimator, "reset"):
            torque_estimator.reset()
        self._index = 0
        self._actual_velocity_mps = np.zeros(2, dtype=float)
        self._actual_tool_tip_pos_m = np.zeros_like(self.desired_tool_tip_pos_m)
        self._actual_tool_tip_pos_m[0] = self.initial_tool_tip_pos_m
        self._normal_force_n = np.zeros(self.sample_count, dtype=float)
        self._penetration_m = np.zeros(self.sample_count, dtype=float)
        self._is_in_contact = np.zeros(self.sample_count, dtype=bool)
        self._joint_torque_nm = np.zeros(
            (self.sample_count, self.simulator.torque_limits_nm.size),
            dtype=float,
        )
        self._torque_ratio = np.zeros(self.sample_count, dtype=float)

    def step(self) -> ContactExecutionSample | None:
        """Advance one deterministic sample and return the current state."""

        if self.is_done:
            return None

        index = self._index
        contact_state = self.simulator.contact_model.compute_contact(
            self._actual_tool_tip_pos_m[index],
            self._actual_velocity_mps,
            self.simulator.surface,
        )
        joint_torque_nm = self.simulator._estimate_torque(
            self._actual_tool_tip_pos_m[index],
            self._actual_velocity_mps,
            contact_state,
        )
        torque_ratio = self.simulator._max_torque_ratio(joint_torque_nm)

        self._normal_force_n[index] = contact_state.normal_force_n
        self._penetration_m[index] = contact_state.penetration_m
        self._is_in_contact[index] = contact_state.is_in_contact
        self._joint_torque_nm[index] = joint_torque_nm
        self._torque_ratio[index] = torque_ratio

        is_last_sample = index == self.sample_count - 1
        sample = ContactExecutionSample(
            sample_index=index,
            time_s=float(self.time_s[index]),
            desired_tool_tip_pos_m=self.desired_tool_tip_pos_m[index].copy(),
            actual_tool_tip_pos_m=self._actual_tool_tip_pos_m[index].copy(),
            normal_force_n=float(contact_state.normal_force_n),
            desired_normal_force_n=float(self.desired_normal_force_n[index]),
            penetration_m=float(contact_state.penetration_m),
            is_in_contact=bool(contact_state.is_in_contact),
            is_excessive_penetration=bool(contact_state.is_excessive_penetration),
            surface_height_m=float(contact_state.surface_height_m),
            joint_torque_nm=joint_torque_nm.copy(),
            torque_ratio=float(torque_ratio),
            is_last_sample=is_last_sample,
        )

        if not is_last_sample:
            command = self.simulator._compute_command(
                self.controller,
                self.desired_tool_tip_pos_m[index],
                self.desired_tool_tip_vel_mps[index],
                self._actual_tool_tip_pos_m[index],
                self._actual_velocity_mps,
                float(self.desired_normal_force_n[index]),
                float(self._normal_force_n[index]),
            )
            dt_s = float(self.time_s[index + 1] - self.time_s[index])
            self._actual_tool_tip_pos_m[index + 1] = (
                self._actual_tool_tip_pos_m[index] + dt_s * command
            )
            self._actual_velocity_mps = command

        self._index += 1
        return sample

    def result(self) -> ContactExecutionResult:
        """Return the completed rollout as a ``ContactExecutionResult``."""

        if not self.is_done:
            raise RuntimeError("cannot build result before all samples are stepped")
        return ContactExecutionResult(
            controller_name=self.controller_name,
            time_s=self.time_s,
            desired_tool_tip_pos_m=self.desired_tool_tip_pos_m,
            actual_tool_tip_pos_m=self._actual_tool_tip_pos_m,
            normal_force_n=self._normal_force_n,
            desired_normal_force_n=self.desired_normal_force_n,
            penetration_m=self._penetration_m,
            is_in_contact=self._is_in_contact,
            joint_torque_nm=self._joint_torque_nm,
            torque_ratio=self._torque_ratio,
        )
