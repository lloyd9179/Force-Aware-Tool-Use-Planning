"""Deterministic kinematic simulator for Phase 3 contact execution."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from force_tool_planning.contact.contact_model import ContactState, PlanarContactModel
from force_tool_planning.contact.surface import Surface2D
from force_tool_planning.control.force_aware_controller import ForceAwareController
from force_tool_planning.control.position_controller import PositionOnlyController
from force_tool_planning.simulation.execution_result import ContactExecutionResult


TorqueEstimator = Callable[[np.ndarray, np.ndarray, ContactState], ArrayLike]


@dataclass(frozen=True)
class ContactExecutionSimulator:
    """Run a deterministic 2D contact execution rollout without ROS2.

    The state update is intentionally kinematic, not dynamic:

    ``actual_pos[t + 1] = actual_pos[t] + dt * commanded_velocity``

    Joint torques are supplied by an optional estimator callback so later Phase
    3 steps can reuse Phase 1 kinematics and torque utilities without baking IK
    or planner details into this simulator.
    """

    surface: Surface2D
    contact_model: PlanarContactModel
    torque_limits_nm: ArrayLike
    torque_estimator: TorqueEstimator | None = None

    def __post_init__(self) -> None:
        limits = self._as_1d_float(self.torque_limits_nm, "torque_limits_nm")
        if limits.size == 0:
            raise ValueError("torque_limits_nm must contain at least one limit")
        if np.any(limits <= 0.0):
            raise ValueError("torque_limits_nm must contain only positive values")
        object.__setattr__(self, "torque_limits_nm", limits)

    def run(
        self,
        controller: PositionOnlyController | ForceAwareController,
        *,
        controller_name: str,
        time_s: ArrayLike,
        desired_tool_tip_pos_m: ArrayLike,
        desired_tool_tip_vel_mps: ArrayLike,
        desired_normal_force_n: ArrayLike | float,
        initial_tool_tip_pos_m: ArrayLike | None = None,
    ) -> ContactExecutionResult:
        """Run ``controller`` over a desired tool-tip trajectory."""

        time = self._as_1d_float(time_s, "time_s")
        if time.size == 0:
            raise ValueError("time_s must contain at least one sample")
        if time.size > 1 and np.any(np.diff(time) <= 0.0):
            raise ValueError("time_s must be strictly increasing")

        desired_pos = self._as_matrix(
            desired_tool_tip_pos_m,
            "desired_tool_tip_pos_m",
            time.size,
            column_count=2,
        )
        desired_vel = self._as_matrix(
            desired_tool_tip_vel_mps,
            "desired_tool_tip_vel_mps",
            time.size,
            column_count=2,
        )
        desired_force = self._as_1d_float_or_scalar(
            desired_normal_force_n,
            "desired_normal_force_n",
            time.size,
        )
        actual_pos = np.zeros_like(desired_pos)
        actual_pos[0] = (
            self._as_vector2(initial_tool_tip_pos_m, "initial_tool_tip_pos_m")
            if initial_tool_tip_pos_m is not None
            else desired_pos[0]
        )

        actual_vel = np.zeros(2, dtype=float)
        normal_force = np.zeros(time.size, dtype=float)
        penetration = np.zeros(time.size, dtype=float)
        is_in_contact = np.zeros(time.size, dtype=bool)
        joint_torque = np.zeros((time.size, self.torque_limits_nm.size), dtype=float)
        torque_ratio = np.zeros(time.size, dtype=float)

        for index in range(time.size):
            contact_state = self.contact_model.compute_contact(
                actual_pos[index],
                actual_vel,
                self.surface,
            )
            normal_force[index] = contact_state.normal_force_n
            penetration[index] = contact_state.penetration_m
            is_in_contact[index] = contact_state.is_in_contact
            joint_torque[index] = self._estimate_torque(
                actual_pos[index],
                actual_vel,
                contact_state,
            )
            torque_ratio[index] = self._max_torque_ratio(joint_torque[index])

            if index == time.size - 1:
                continue

            command = self._compute_command(
                controller,
                desired_pos[index],
                desired_vel[index],
                actual_pos[index],
                actual_vel,
                desired_force[index],
                normal_force[index],
            )
            dt_s = float(time[index + 1] - time[index])
            actual_pos[index + 1] = actual_pos[index] + dt_s * command
            actual_vel = command

        return ContactExecutionResult(
            controller_name=controller_name,
            time_s=time,
            desired_tool_tip_pos_m=desired_pos,
            actual_tool_tip_pos_m=actual_pos,
            normal_force_n=normal_force,
            desired_normal_force_n=desired_force,
            penetration_m=penetration,
            is_in_contact=is_in_contact,
            joint_torque_nm=joint_torque,
            torque_ratio=torque_ratio,
        )

    def _compute_command(
        self,
        controller: PositionOnlyController | ForceAwareController,
        desired_pos_m: np.ndarray,
        desired_vel_mps: np.ndarray,
        current_pos_m: np.ndarray,
        current_vel_mps: np.ndarray,
        desired_normal_force_n: float,
        measured_normal_force_n: float,
    ) -> np.ndarray:
        if isinstance(controller, PositionOnlyController):
            return controller.compute_tool_tip_command(
                desired_pos_m,
                desired_vel_mps,
                current_pos_m,
                current_vel_mps,
            )
        if isinstance(controller, ForceAwareController):
            x_m = float(current_pos_m[0])
            return controller.compute_tool_tip_command(
                desired_pos_m,
                desired_vel_mps,
                current_pos_m,
                current_vel_mps,
                self.surface.tangent(x_m),
                self.surface.normal(x_m),
                desired_normal_force_n,
                measured_normal_force_n,
            )
        raise TypeError("controller must be PositionOnlyController or ForceAwareController")

    def _estimate_torque(
        self,
        actual_pos_m: np.ndarray,
        actual_vel_mps: np.ndarray,
        contact_state: ContactState,
    ) -> np.ndarray:
        if self.torque_estimator is None:
            return np.zeros_like(self.torque_limits_nm)

        torque = self._as_1d_float(
            self.torque_estimator(actual_pos_m, actual_vel_mps, contact_state),
            "torque_estimator result",
        )
        if torque.shape != self.torque_limits_nm.shape:
            raise ValueError(
                "torque_estimator result must have shape "
                f"{self.torque_limits_nm.shape}"
            )
        return torque

    def _max_torque_ratio(self, torque_nm: np.ndarray) -> float:
        return float(np.max(np.abs(torque_nm) / self.torque_limits_nm))

    @staticmethod
    def _as_vector2(value: ArrayLike, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (2,):
            raise ValueError(f"{name} must have shape (2,)")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array.copy()

    @staticmethod
    def _as_1d_float(value: ArrayLike, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.ndim != 1:
            raise ValueError(f"{name} must be a 1D array")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array.copy()

    @classmethod
    def _as_1d_float_or_scalar(
        cls,
        value: ArrayLike | float,
        name: str,
        expected_length: int,
    ) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.ndim == 0:
            scalar = float(array)
            if not np.isfinite(scalar):
                raise ValueError(f"{name} must be finite")
            return np.full(expected_length, scalar, dtype=float)
        vector = cls._as_1d_float(array, name)
        if vector.shape[0] != expected_length:
            raise ValueError(f"{name} must have length {expected_length}")
        return vector

    @staticmethod
    def _as_matrix(
        value: ArrayLike,
        name: str,
        expected_rows: int,
        column_count: int,
    ) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.ndim != 2:
            raise ValueError(f"{name} must be a 2D array")
        if array.shape != (expected_rows, column_count):
            raise ValueError(f"{name} must have shape {(expected_rows, column_count)}")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array.copy()
