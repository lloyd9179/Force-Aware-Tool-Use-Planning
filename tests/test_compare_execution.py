from pathlib import Path

import numpy as np
import pytest
import yaml

from force_tool_planning.analysis.compare_execution import (
    DEFAULT_PHASE3_CONFIG,
    build_desired_contact_trajectory,
    compare_controllers,
    load_phase3_config,
)

ROOT = Path(__file__).resolve().parents[1]
PHASE1_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


def _load_default_config() -> dict:
    with DEFAULT_PHASE3_CONFIG.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _write_config(tmp_path: Path, config: dict) -> Path:
    path = tmp_path / "phase3.yaml"
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream)
    return path


def test_default_phase3_comparison_runs_both_controllers() -> None:
    comparison = compare_controllers(DEFAULT_PHASE3_CONFIG)

    assert comparison.position_only.controller_name == "position_only"
    assert comparison.force_aware.controller_name == "force_aware"
    assert comparison.position_only.sample_count == comparison.force_aware.sample_count
    assert comparison.position_only.metrics["max_torque_ratio"] > 1.0
    assert comparison.force_aware.metrics["max_torque_ratio"] > 0.0
    assert comparison.force_aware.metrics["max_torque_ratio"] < 1.0
    assert comparison.position_only.metrics["success"] is False
    assert comparison.force_aware.metrics["force_rmse_n"] < comparison.position_only.metrics[
        "force_rmse_n"
    ]
    assert "torque_limit_exceeded" in comparison.position_only.failure_reasons
    assert "excessive_force" in comparison.position_only.failure_reasons


def test_desired_contact_trajectory_uses_speed_and_planned_penetration() -> None:
    config = load_phase3_config(DEFAULT_PHASE3_CONFIG)
    trajectory = build_desired_contact_trajectory(config)

    task = config["task"]
    contact = config["contact_model"]
    expected_y = (
        task["planned_surface_height_m"]
        - task["desired_normal_force_n"] / contact["normal_stiffness_n_per_m"]
    )

    assert trajectory.time_s[0] == pytest.approx(0.0)
    assert trajectory.time_s[-1] == pytest.approx(task["duration_s"])
    assert trajectory.desired_tool_tip_pos_m[0, 0] == pytest.approx(
        task["tangential_start_m"]
    )
    assert trajectory.desired_tool_tip_pos_m[-1, 0] == pytest.approx(
        task["tangential_end_m"]
    )
    assert np.allclose(trajectory.desired_tool_tip_pos_m[:, 1], expected_y)
    assert trajectory.desired_tool_tip_vel_mps[0, 0] == pytest.approx(
        task["desired_tangential_speed_mps"]
    )
    assert trajectory.desired_tool_tip_vel_mps[-1, 0] == pytest.approx(0.0)


def test_phase3_config_validation_rejects_missing_sections(tmp_path: Path) -> None:
    config = _load_default_config()
    del config["controllers"]

    with pytest.raises(ValueError, match="missing required section: controllers"):
        load_phase3_config(_write_config(tmp_path, config))


def test_phase3_config_can_resolve_installed_sibling_phase1_config(
    tmp_path: Path,
) -> None:
    phase3_config = tmp_path / "phase3_contact_execution.yaml"
    phase1_config = tmp_path / "demo_planar_arm.yaml"
    phase3_config.write_text(
        DEFAULT_PHASE3_CONFIG.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    phase1_config.write_text(
        PHASE1_CONFIG.read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    loaded = load_phase3_config(phase3_config)

    assert loaded["arm"]["source_config"] == "configs/demo_planar_arm.yaml"
