import numpy as np
import pytest

from force_tool_planning.simulation.execution_result import ContactExecutionResult


def _valid_result(**overrides: object) -> ContactExecutionResult:
    values: dict[str, object] = {
        "controller_name": "force_aware",
        "time_s": np.array([0.0, 0.1, 0.2]),
        "desired_tool_tip_pos_m": np.array(
            [
                [0.0, 0.01],
                [0.1, 0.01],
                [0.2, 0.01],
            ]
        ),
        "actual_tool_tip_pos_m": np.array(
            [
                [0.0, 0.0],
                [0.1, 0.005],
                [0.2, 0.008],
            ]
        ),
        "normal_force_n": np.array([4.8, 5.0, 5.1]),
        "desired_normal_force_n": 5.0,
        "penetration_m": np.array([0.012, 0.013, 0.0125]),
        "is_in_contact": np.array([True, True, True]),
        "joint_torque_nm": np.array(
            [
                [0.2, 0.1, 0.05],
                [0.3, 0.1, 0.06],
                [0.25, 0.12, 0.05],
            ]
        ),
        "torque_ratio": np.array([0.5, 0.7, 0.6]),
        "metrics": {"force_rmse_n": 0.15, "success": True},
        "failure_reasons": (),
    }
    values.update(overrides)
    return ContactExecutionResult(**values)


def test_contact_execution_result_stores_time_series_and_metadata() -> None:
    result = _valid_result(failure_reasons=("contact_loss",))

    assert result.controller_name == "force_aware"
    assert result.sample_count == 3
    assert result.joint_count == 3
    assert result.duration_s == pytest.approx(0.2)
    assert result.max_torque_ratio == pytest.approx(0.7)
    np.testing.assert_allclose(result.desired_normal_force_n, np.array([5.0, 5.0, 5.0]))
    assert result.metrics == {"force_rmse_n": pytest.approx(0.15), "success": True}
    assert result.failure_reasons == ("contact_loss",)


def test_contact_execution_result_accepts_time_varying_desired_force() -> None:
    result = _valid_result(desired_normal_force_n=np.array([4.0, 5.0, 6.0]))

    np.testing.assert_allclose(result.desired_normal_force_n, np.array([4.0, 5.0, 6.0]))


def test_contact_execution_result_copies_input_arrays() -> None:
    time_s = np.array([0.0, 0.1, 0.2])

    result = _valid_result(time_s=time_s)
    time_s[1] = 99.0

    np.testing.assert_allclose(result.time_s, np.array([0.0, 0.1, 0.2]))


def test_contact_execution_result_rejects_invalid_controller_name() -> None:
    with pytest.raises(ValueError, match="controller_name"):
        _valid_result(controller_name=" ")


def test_contact_execution_result_rejects_invalid_time() -> None:
    with pytest.raises(ValueError, match="time_s"):
        _valid_result(time_s=np.array([]))

    with pytest.raises(ValueError, match="time_s"):
        _valid_result(time_s=np.array([0.0, 0.2, 0.1]))


def test_contact_execution_result_rejects_shape_mismatches() -> None:
    with pytest.raises(ValueError, match="desired_tool_tip_pos_m"):
        _valid_result(desired_tool_tip_pos_m=np.zeros((3, 3)))

    with pytest.raises(ValueError, match="normal_force_n"):
        _valid_result(normal_force_n=np.array([1.0, 2.0]))

    with pytest.raises(ValueError, match="joint_torque_nm"):
        _valid_result(joint_torque_nm=np.array([0.1, 0.2, 0.3]))


def test_contact_execution_result_rejects_invalid_metrics_and_failures() -> None:
    with pytest.raises(ValueError, match="metrics values"):
        _valid_result(metrics={"force_rmse_n": np.inf})

    with pytest.raises(ValueError, match="failure_reasons"):
        _valid_result(failure_reasons=("contact_loss", ""))
