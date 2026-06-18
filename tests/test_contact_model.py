import numpy as np
import pytest

from force_tool_planning.contact.contact_model import ContactState, PlanarContactModel
from force_tool_planning.contact.surface import Surface2D


def make_model() -> PlanarContactModel:
    return PlanarContactModel(
        normal_stiffness_n_per_m=400.0,
        normal_damping_n_s_per_m=5.0,
        friction_coefficient=0.4,
        max_penetration_m=0.05,
    )


def test_no_contact_when_tool_tip_is_above_surface() -> None:
    surface = Surface2D("flat", planned_height_m=0.0)
    state = make_model().compute_contact(
        tool_tip_pos_m=np.array([0.0, 0.10]),
        tool_tip_vel_mps=np.zeros(2),
        surface=surface,
    )

    assert isinstance(state, ContactState)
    assert state.surface_height_m == pytest.approx(0.0)
    assert state.penetration_m == pytest.approx(-0.10)
    assert state.normal_force_n == pytest.approx(0.0)
    assert state.friction_limit_n == pytest.approx(0.0)
    assert state.is_in_contact is False
    assert state.is_excessive_penetration is False


def test_positive_normal_force_when_tool_tip_penetrates_surface() -> None:
    surface = Surface2D("flat", planned_height_m=0.0)
    state = make_model().compute_contact(
        tool_tip_pos_m=np.array([0.0, -0.02]),
        tool_tip_vel_mps=np.zeros(2),
        surface=surface,
    )

    assert state.penetration_m == pytest.approx(0.02)
    assert state.normal_force_n == pytest.approx(8.0)
    assert state.is_in_contact is True


def test_normal_force_increases_with_penetration() -> None:
    surface = Surface2D("flat", planned_height_m=0.0)
    model = make_model()

    shallow = model.compute_contact([0.0, -0.01], [0.0, 0.0], surface)
    deep = model.compute_contact([0.0, -0.03], [0.0, 0.0], surface)

    assert deep.normal_force_n > shallow.normal_force_n
    assert shallow.normal_force_n == pytest.approx(4.0)
    assert deep.normal_force_n == pytest.approx(12.0)


def test_normal_damping_increases_force_when_tool_moves_into_surface() -> None:
    surface = Surface2D("flat", planned_height_m=0.0)
    model = make_model()

    stationary = model.compute_contact([0.0, -0.02], [0.0, 0.0], surface)
    moving_down = model.compute_contact([0.0, -0.02], [0.0, -0.2], surface)

    assert moving_down.normal_force_n > stationary.normal_force_n
    assert moving_down.normal_force_n == pytest.approx(9.0)


def test_excessive_penetration_flag_is_triggered() -> None:
    surface = Surface2D("flat", planned_height_m=0.0)
    state = make_model().compute_contact([0.0, -0.06], [0.0, 0.0], surface)

    assert state.penetration_m == pytest.approx(0.06)
    assert state.is_excessive_penetration is True


def test_friction_limit_equals_mu_times_normal_force() -> None:
    surface = Surface2D("flat", planned_height_m=0.0)
    state = make_model().compute_contact([0.0, -0.025], [0.0, 0.0], surface)

    assert state.normal_force_n == pytest.approx(10.0)
    assert state.friction_limit_n == pytest.approx(4.0)
    assert state.as_dict()["friction_limit_n"] == pytest.approx(4.0)


def test_contact_model_validation_rejects_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="normal_stiffness"):
        PlanarContactModel(0.0, 5.0, 0.4, 0.05)

    with pytest.raises(ValueError, match="normal_damping"):
        PlanarContactModel(400.0, -1.0, 0.4, 0.05)

    with pytest.raises(ValueError, match="friction_coefficient"):
        PlanarContactModel(400.0, 5.0, -0.1, 0.05)

    with pytest.raises(ValueError, match="max_penetration"):
        PlanarContactModel(400.0, 5.0, 0.4, -0.01)

    surface = Surface2D("flat", planned_height_m=0.0)
    with pytest.raises(ValueError, match="tool_tip_pos_m"):
        make_model().compute_contact([0.0, 0.0, 0.0], [0.0, 0.0], surface)

    with pytest.raises(ValueError, match="tool_tip_vel_mps"):
        make_model().compute_contact([0.0, 0.0], [np.nan, 0.0], surface)
