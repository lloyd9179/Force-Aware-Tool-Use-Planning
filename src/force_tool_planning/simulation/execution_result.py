"""Structured execution result for Phase 3 contact simulations."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike


MetricValue = float | int | bool


@dataclass(frozen=True)
class ContactExecutionResult:
    """Time-series result produced by a Phase 3 contact execution run."""

    controller_name: str
    time_s: ArrayLike
    desired_tool_tip_pos_m: ArrayLike
    actual_tool_tip_pos_m: ArrayLike
    normal_force_n: ArrayLike
    desired_normal_force_n: ArrayLike | float
    penetration_m: ArrayLike
    is_in_contact: ArrayLike
    joint_torque_nm: ArrayLike
    torque_ratio: ArrayLike
    metrics: Mapping[str, MetricValue] = field(default_factory=dict)
    failure_reasons: Sequence[str] = ()

    def __post_init__(self) -> None:
        controller_name = str(self.controller_name).strip()
        if not controller_name:
            raise ValueError("controller_name must be non-empty")
        object.__setattr__(self, "controller_name", controller_name)

        time_s = self._as_1d_float(self.time_s, "time_s")
        if time_s.size == 0:
            raise ValueError("time_s must contain at least one sample")
        if time_s.size > 1 and np.any(np.diff(time_s) <= 0.0):
            raise ValueError("time_s must be strictly increasing")
        sample_count = time_s.size

        desired_pos = self._as_matrix(
            self.desired_tool_tip_pos_m,
            "desired_tool_tip_pos_m",
            sample_count,
            column_count=2,
        )
        actual_pos = self._as_matrix(
            self.actual_tool_tip_pos_m,
            "actual_tool_tip_pos_m",
            sample_count,
            column_count=2,
        )
        normal_force = self._as_1d_float(
            self.normal_force_n, "normal_force_n", sample_count
        )
        desired_normal_force = self._as_1d_float_or_scalar(
            self.desired_normal_force_n,
            "desired_normal_force_n",
            sample_count,
        )
        penetration = self._as_1d_float(self.penetration_m, "penetration_m", sample_count)
        is_in_contact = self._as_1d_bool(
            self.is_in_contact, "is_in_contact", sample_count
        )
        joint_torque = self._as_matrix(
            self.joint_torque_nm,
            "joint_torque_nm",
            sample_count,
        )
        torque_ratio = self._as_1d_float(self.torque_ratio, "torque_ratio", sample_count)
        metrics = self._validated_metrics(self.metrics)
        failure_reasons = self._validated_failure_reasons(self.failure_reasons)

        object.__setattr__(self, "time_s", time_s)
        object.__setattr__(self, "desired_tool_tip_pos_m", desired_pos)
        object.__setattr__(self, "actual_tool_tip_pos_m", actual_pos)
        object.__setattr__(self, "normal_force_n", normal_force)
        object.__setattr__(self, "desired_normal_force_n", desired_normal_force)
        object.__setattr__(self, "penetration_m", penetration)
        object.__setattr__(self, "is_in_contact", is_in_contact)
        object.__setattr__(self, "joint_torque_nm", joint_torque)
        object.__setattr__(self, "torque_ratio", torque_ratio)
        object.__setattr__(self, "metrics", metrics)
        object.__setattr__(self, "failure_reasons", failure_reasons)

    @property
    def sample_count(self) -> int:
        """Return the number of stored time samples."""

        return int(self.time_s.size)

    @property
    def joint_count(self) -> int:
        """Return the number of stored joint torque columns."""

        return int(self.joint_torque_nm.shape[1])

    @property
    def duration_s(self) -> float:
        """Return elapsed duration covered by the result in seconds."""

        return float(self.time_s[-1] - self.time_s[0])

    @property
    def max_torque_ratio(self) -> float:
        """Return the maximum stored torque-limit ratio."""

        return float(np.max(self.torque_ratio))

    @staticmethod
    def _as_1d_float(
        value: ArrayLike,
        name: str,
        expected_length: int | None = None,
    ) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.ndim != 1:
            raise ValueError(f"{name} must be a 1D array")
        if expected_length is not None and array.shape[0] != expected_length:
            raise ValueError(f"{name} must have length {expected_length}")
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
        return cls._as_1d_float(array, name, expected_length)

    @staticmethod
    def _as_1d_bool(value: ArrayLike, name: str, expected_length: int) -> np.ndarray:
        array = np.asarray(value)
        if array.ndim != 1:
            raise ValueError(f"{name} must be a 1D array")
        if array.shape[0] != expected_length:
            raise ValueError(f"{name} must have length {expected_length}")
        return array.astype(bool, copy=True)

    @staticmethod
    def _as_matrix(
        value: ArrayLike,
        name: str,
        expected_rows: int,
        column_count: int | None = None,
    ) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.ndim != 2:
            raise ValueError(f"{name} must be a 2D array")
        if array.shape[0] != expected_rows:
            raise ValueError(f"{name} must have {expected_rows} rows")
        if column_count is not None and array.shape[1] != column_count:
            raise ValueError(f"{name} must have {column_count} columns")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        return array.copy()

    @staticmethod
    def _validated_metrics(metrics: Mapping[str, MetricValue]) -> dict[str, MetricValue]:
        validated: dict[str, MetricValue] = {}
        for key, value in dict(metrics).items():
            if not isinstance(key, str) or not key:
                raise ValueError("metrics keys must be non-empty strings")
            if isinstance(value, (bool, np.bool_)):
                validated[key] = bool(value)
                continue
            if not isinstance(value, (int, float, np.integer, np.floating)):
                raise ValueError("metrics values must be numeric or boolean")
            numeric_value = float(value)
            if not np.isfinite(numeric_value):
                raise ValueError("metrics values must be finite")
            validated[key] = numeric_value
        return validated

    @staticmethod
    def _validated_failure_reasons(failure_reasons: Sequence[str]) -> tuple[str, ...]:
        reasons = tuple(failure_reasons)
        for reason in reasons:
            if not isinstance(reason, str) or not reason:
                raise ValueError("failure_reasons must contain non-empty strings")
        return reasons
