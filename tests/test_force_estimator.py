import numpy as np
import pytest

from force_tool_planning.contact.force_estimator import estimate_contact_wrench_2d
from force_tool_planning.jacobian import planar_jacobian
from force_tool_planning.kinematics import ArmModel
from force_tool_planning.torque import joint_torques_from_wrench


def test_estimate_contact_wrench_uses_origin_by_default() -> None:
    wrench = estimate_contact_wrench_2d(normal_force_n=5.0)

    np.testing.assert_allclose(wrench, np.array([0.0, 5.0, 0.0]))


def test_estimate_contact_wrench_includes_tangential_force_and_moment() -> None:
    wrench = estimate_contact_wrench_2d(
        normal_force_n=5.0,
        tangential_force_n=2.0,
        contact_point_m=np.array([0.3, -0.1]),
    )

    np.testing.assert_allclose(wrench, np.array([2.0, 5.0, 1.7]))


def test_estimated_contact_wrench_connects_to_existing_torque_utility() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 1.0])
    jacobian = planar_jacobian(arm, [0.0, 0.0, 0.0])
    wrench = estimate_contact_wrench_2d(
        normal_force_n=10.0,
        tangential_force_n=0.0,
        contact_point_m=[0.2, 0.0],
    )

    torque_nm = joint_torques_from_wrench(jacobian, wrench)

    np.testing.assert_allclose(wrench, np.array([0.0, 10.0, 2.0]))
    np.testing.assert_allclose(torque_nm, np.array([32.0, 22.0, 12.0]))


def test_force_estimator_validation_rejects_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="normal_force_n"):
        estimate_contact_wrench_2d(np.nan)

    with pytest.raises(ValueError, match="tangential_force_n"):
        estimate_contact_wrench_2d(1.0, tangential_force_n=np.inf)

    with pytest.raises(ValueError, match="contact_point_m"):
        estimate_contact_wrench_2d(1.0, contact_point_m=[0.0, 0.0, 0.0])

    with pytest.raises(ValueError, match="contact_point_m"):
        estimate_contact_wrench_2d(1.0, contact_point_m=[0.0, np.nan])
