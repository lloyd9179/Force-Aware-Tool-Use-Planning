import numpy as np
import pytest

from force_tool_planning.contact.surface import Surface2D


def test_flat_surface_height_is_constant() -> None:
    surface = Surface2D("flat", planned_height_m=0.2, height_offset_m=0.03)

    assert surface.height(-1.0) == pytest.approx(0.23)
    assert surface.height(0.0) == pytest.approx(0.23)
    assert surface.height(1.0) == pytest.approx(0.23)


def test_sinusoidal_surface_height_changes_with_x() -> None:
    surface = Surface2D(
        "sinusoidal",
        planned_height_m=0.0,
        height_offset_m=0.03,
        amplitude_m=0.02,
        frequency_cycles_per_m=1.0,
    )

    assert surface.height(0.0) == pytest.approx(0.03)
    assert surface.height(0.25) == pytest.approx(0.05)
    assert surface.height(0.75) == pytest.approx(0.01)


def test_flat_surface_tangent_and_normal_match_expected_axes() -> None:
    surface = Surface2D("flat", planned_height_m=0.0)

    np.testing.assert_allclose(surface.tangent(0.0), np.array([1.0, 0.0]))
    np.testing.assert_allclose(surface.normal(0.0), np.array([0.0, 1.0]))


def test_tangent_and_normal_are_unit_and_orthogonal() -> None:
    surface = Surface2D(
        "sinusoidal",
        planned_height_m=0.0,
        amplitude_m=0.05,
        frequency_cycles_per_m=2.0,
    )

    tangent = surface.tangent(0.13)
    normal = surface.normal(0.13)

    assert np.linalg.norm(tangent) == pytest.approx(1.0)
    assert np.linalg.norm(normal) == pytest.approx(1.0)
    assert float(np.dot(tangent, normal)) == pytest.approx(0.0, abs=1e-12)
    assert normal[1] > 0.0


def test_surface_validation_rejects_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="surface_type"):
        Surface2D("rough", planned_height_m=0.0)

    with pytest.raises(ValueError, match="planned_height_m"):
        Surface2D("flat", planned_height_m=np.nan)

    with pytest.raises(ValueError, match="frequency_cycles_per_m"):
        Surface2D("sinusoidal", planned_height_m=0.0, frequency_cycles_per_m=-1.0)

    surface = Surface2D("flat", planned_height_m=0.0)
    with pytest.raises(ValueError, match="x_m"):
        surface.height(np.inf)
