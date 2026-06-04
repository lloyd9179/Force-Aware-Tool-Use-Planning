import numpy as np
import pytest

from force_tool_planning.jacobian import planar_jacobian, translational_jacobian_xy
from force_tool_planning.kinematics import ArmModel
from force_tool_planning.torque import (
    check_torque_limits,
    is_torque_feasible,
    joint_torques_from_force,
    joint_torques_from_wrench,
)


def test_joint_torques_from_force_use_j_transpose_f() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 1.0])
    jacobian_xy = translational_jacobian_xy(arm, [0.0, 0.0, 0.0])

    torque_nm = joint_torques_from_force(jacobian_xy, [0.0, 10.0])

    np.testing.assert_allclose(torque_nm, np.array([30.0, 20.0, 10.0]))


def test_joint_torques_from_planar_wrench_include_moment_row() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 1.0])
    jacobian = planar_jacobian(arm, [0.0, 0.0, 0.0])

    torque_nm = joint_torques_from_wrench(jacobian, [0.0, 10.0, 2.0])

    np.testing.assert_allclose(torque_nm, np.array([32.0, 22.0, 12.0]))


def test_torque_limit_check_reports_margins_and_violations() -> None:
    result = check_torque_limits([30.0, -20.0, 10.0], [30.0, 19.0, 10.0])

    assert not result.feasible
    np.testing.assert_allclose(result.margin_nm, np.array([0.0, -1.0, 0.0]))
    np.testing.assert_array_equal(result.violating_joint_indices, np.array([1]))


def test_is_torque_feasible_accepts_boundary_values() -> None:
    assert is_torque_feasible([30.0, -20.0, 10.0], [30.0, 20.0, 10.0])


def test_torque_helpers_validate_dimensions() -> None:
    with pytest.raises(ValueError, match="row count"):
        joint_torques_from_wrench(np.zeros((2, 3)), np.zeros(3))

    with pytest.raises(ValueError, match="torque_limits_nm"):
        check_torque_limits([1.0, 2.0], [1.0])

    with pytest.raises(ValueError, match="positive"):
        check_torque_limits([1.0, 2.0], [1.0, 0.0])
