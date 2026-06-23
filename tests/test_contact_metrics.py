import numpy as np
import pytest

from force_tool_planning.contact.contact_metrics import (
    ContactMetricThresholds,
    compute_contact_metrics,
    result_with_contact_metrics,
)
from force_tool_planning.simulation.execution_result import ContactExecutionResult


def _thresholds() -> ContactMetricThresholds:
    return ContactMetricThresholds(
        contact_loss_force_threshold_n=0.2,
        excessive_force_threshold_n=12.0,
        contact_loss_fraction_threshold=0.25,
        torque_warning_ratio=0.9,
        torque_failure_ratio=1.0,
        max_penetration_m=0.05,
    )


def _result(**overrides: object) -> ContactExecutionResult:
    values: dict[str, object] = {
        "controller_name": "force_aware",
        "time_s": np.array([0.0, 0.1, 0.2, 0.3]),
        "desired_tool_tip_pos_m": np.zeros((4, 2)),
        "actual_tool_tip_pos_m": np.zeros((4, 2)),
        "normal_force_n": np.array([4.0, 5.0, 6.0, 5.0]),
        "desired_normal_force_n": np.array([5.0, 5.0, 5.0, 5.0]),
        "penetration_m": np.array([0.01, 0.012, 0.011, 0.013]),
        "is_in_contact": np.array([True, True, True, True]),
        "joint_torque_nm": np.zeros((4, 3)),
        "torque_ratio": np.array([0.4, 0.5, 0.8, 0.7]),
    }
    values.update(overrides)
    return ContactExecutionResult(**values)


def test_compute_contact_metrics_reports_successful_execution() -> None:
    metrics = compute_contact_metrics(_result(), _thresholds())

    assert metrics.force_rmse_n == pytest.approx(np.sqrt(0.5))
    assert metrics.contact_loss_fraction == pytest.approx(0.0)
    assert metrics.max_penetration_m == pytest.approx(0.013)
    assert metrics.max_torque_ratio == pytest.approx(0.8)
    assert metrics.torque_warning_count == 0
    assert metrics.torque_violation_count == 0
    assert metrics.excessive_force_count == 0
    assert metrics.excessive_penetration_count == 0
    assert metrics.success is True
    assert metrics.failure_reasons == ()


def test_compute_contact_metrics_counts_failures_and_reasons() -> None:
    result = _result(
        normal_force_n=np.array([0.0, 13.0, 0.1, 5.0]),
        penetration_m=np.array([0.0, 0.02, 0.06, 0.01]),
        is_in_contact=np.array([False, True, True, True]),
        torque_ratio=np.array([0.95, 1.2, 0.4, 0.5]),
    )

    metrics = compute_contact_metrics(result, _thresholds())

    assert metrics.contact_loss_fraction == pytest.approx(0.5)
    assert metrics.max_penetration_m == pytest.approx(0.06)
    assert metrics.max_torque_ratio == pytest.approx(1.2)
    assert metrics.torque_warning_count == 2
    assert metrics.torque_violation_count == 1
    assert metrics.excessive_force_count == 1
    assert metrics.excessive_penetration_count == 1
    assert metrics.success is False
    assert metrics.failure_reasons == (
        "contact_loss_fraction_exceeded",
        "torque_limit_exceeded",
        "excessive_penetration",
        "excessive_force",
    )


def test_result_with_contact_metrics_attaches_metrics_to_result_copy() -> None:
    result = _result()

    evaluated = result_with_contact_metrics(result, _thresholds())

    assert evaluated is not result
    assert evaluated.metrics["success"] is True
    assert evaluated.metrics["force_rmse_n"] == pytest.approx(np.sqrt(0.5))
    assert evaluated.failure_reasons == ()
    np.testing.assert_allclose(evaluated.normal_force_n, result.normal_force_n)


def test_contact_metric_threshold_validation_rejects_invalid_values() -> None:
    with pytest.raises(ValueError, match="contact_loss_fraction_threshold"):
        ContactMetricThresholds(
            contact_loss_force_threshold_n=0.2,
            excessive_force_threshold_n=12.0,
            contact_loss_fraction_threshold=1.1,
            torque_warning_ratio=0.9,
            torque_failure_ratio=1.0,
            max_penetration_m=0.05,
        )

    with pytest.raises(ValueError, match="torque_warning_ratio"):
        ContactMetricThresholds(
            contact_loss_force_threshold_n=0.2,
            excessive_force_threshold_n=12.0,
            contact_loss_fraction_threshold=0.2,
            torque_warning_ratio=1.1,
            torque_failure_ratio=1.0,
            max_penetration_m=0.05,
        )
