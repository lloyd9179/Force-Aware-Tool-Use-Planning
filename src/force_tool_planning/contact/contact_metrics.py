"""Metrics for Phase 3 contact execution results."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from force_tool_planning.simulation.execution_result import ContactExecutionResult


@dataclass(frozen=True)
class ContactMetricThresholds:
    """Thresholds used to classify Phase 3 contact execution success."""

    contact_loss_force_threshold_n: float
    excessive_force_threshold_n: float
    contact_loss_fraction_threshold: float
    torque_warning_ratio: float
    torque_failure_ratio: float
    max_penetration_m: float

    def __post_init__(self) -> None:
        self._validate_nonnegative(
            self.contact_loss_force_threshold_n,
            "contact_loss_force_threshold_n",
        )
        self._validate_positive(
            self.excessive_force_threshold_n,
            "excessive_force_threshold_n",
        )
        self._validate_fraction(
            self.contact_loss_fraction_threshold,
            "contact_loss_fraction_threshold",
        )
        self._validate_nonnegative(self.torque_warning_ratio, "torque_warning_ratio")
        self._validate_positive(self.torque_failure_ratio, "torque_failure_ratio")
        self._validate_nonnegative(self.max_penetration_m, "max_penetration_m")
        if self.torque_warning_ratio > self.torque_failure_ratio:
            raise ValueError("torque_warning_ratio must not exceed torque_failure_ratio")

    @staticmethod
    def _validate_nonnegative(value: float, name: str) -> None:
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be non-negative and finite")

    @staticmethod
    def _validate_positive(value: float, name: str) -> None:
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")

    @staticmethod
    def _validate_fraction(value: float, name: str) -> None:
        if not np.isfinite(value) or value < 0.0 or value > 1.0:
            raise ValueError(f"{name} must be in [0, 1]")


@dataclass(frozen=True)
class ContactExecutionMetrics:
    """Computed metrics and failure reasons for one contact execution result."""

    force_rmse_n: float
    contact_loss_fraction: float
    max_penetration_m: float
    max_torque_ratio: float
    torque_warning_count: int
    torque_violation_count: int
    excessive_force_count: int
    excessive_penetration_count: int
    success: bool
    failure_reasons: tuple[str, ...]

    def as_dict(self) -> dict[str, float | int | bool]:
        """Return numeric and boolean metrics for result metadata."""

        values = asdict(self)
        values.pop("failure_reasons")
        return values


def compute_contact_metrics(
    result: ContactExecutionResult,
    thresholds: ContactMetricThresholds,
) -> ContactExecutionMetrics:
    """Compute Phase 3 contact execution metrics for ``result``."""

    force_error_n = result.normal_force_n - result.desired_normal_force_n
    force_rmse_n = float(np.sqrt(np.mean(force_error_n**2)))

    contact_lost = np.logical_or(
        ~result.is_in_contact,
        result.normal_force_n <= thresholds.contact_loss_force_threshold_n,
    )
    contact_loss_fraction = float(np.mean(contact_lost))
    max_penetration_m = float(np.max(result.penetration_m))
    max_torque_ratio = float(np.max(result.torque_ratio))
    torque_warning_count = int(
        np.count_nonzero(result.torque_ratio >= thresholds.torque_warning_ratio)
    )
    torque_violation_count = int(
        np.count_nonzero(result.torque_ratio > thresholds.torque_failure_ratio)
    )
    excessive_force_count = int(
        np.count_nonzero(result.normal_force_n > thresholds.excessive_force_threshold_n)
    )
    excessive_penetration_count = int(
        np.count_nonzero(result.penetration_m > thresholds.max_penetration_m)
    )

    failure_reasons = _failure_reasons(
        contact_loss_fraction=contact_loss_fraction,
        max_torque_ratio=max_torque_ratio,
        excessive_force_count=excessive_force_count,
        excessive_penetration_count=excessive_penetration_count,
        thresholds=thresholds,
    )

    return ContactExecutionMetrics(
        force_rmse_n=force_rmse_n,
        contact_loss_fraction=contact_loss_fraction,
        max_penetration_m=max_penetration_m,
        max_torque_ratio=max_torque_ratio,
        torque_warning_count=torque_warning_count,
        torque_violation_count=torque_violation_count,
        excessive_force_count=excessive_force_count,
        excessive_penetration_count=excessive_penetration_count,
        success=len(failure_reasons) == 0,
        failure_reasons=failure_reasons,
    )


def result_with_contact_metrics(
    result: ContactExecutionResult,
    thresholds: ContactMetricThresholds,
) -> ContactExecutionResult:
    """Return ``result`` data with computed metrics and failure reasons attached."""

    metrics = compute_contact_metrics(result, thresholds)
    return ContactExecutionResult(
        controller_name=result.controller_name,
        time_s=result.time_s,
        desired_tool_tip_pos_m=result.desired_tool_tip_pos_m,
        actual_tool_tip_pos_m=result.actual_tool_tip_pos_m,
        normal_force_n=result.normal_force_n,
        desired_normal_force_n=result.desired_normal_force_n,
        penetration_m=result.penetration_m,
        is_in_contact=result.is_in_contact,
        joint_torque_nm=result.joint_torque_nm,
        torque_ratio=result.torque_ratio,
        metrics=metrics.as_dict(),
        failure_reasons=metrics.failure_reasons,
    )


def _failure_reasons(
    *,
    contact_loss_fraction: float,
    max_torque_ratio: float,
    excessive_force_count: int,
    excessive_penetration_count: int,
    thresholds: ContactMetricThresholds,
) -> tuple[str, ...]:
    reasons: list[str] = []
    if contact_loss_fraction > thresholds.contact_loss_fraction_threshold:
        reasons.append("contact_loss_fraction_exceeded")
    if max_torque_ratio > thresholds.torque_failure_ratio:
        reasons.append("torque_limit_exceeded")
    if excessive_penetration_count > 0:
        reasons.append("excessive_penetration")
    if excessive_force_count > 0:
        reasons.append("excessive_force")
    return tuple(reasons)
