from pathlib import Path

import numpy as np
import pytest
import yaml

from force_tool_planning.config import (
    arm_from_config,
    grasps_from_config,
    load_demo_config,
    task_from_config,
)
from force_tool_planning.planner import plan_baseline, plan_force_aware

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


def load_default_as_dict() -> dict:
    with DEFAULT_CONFIG.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def write_config(tmp_path: Path, config: dict) -> Path:
    config_path = tmp_path / "config.yaml"
    with config_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream)
    return config_path


def test_default_config_builds_expected_phase1_objects() -> None:
    config = load_demo_config(DEFAULT_CONFIG)
    arm = arm_from_config(config)
    task = task_from_config(config)
    grasps = grasps_from_config(config)

    assert arm.n_joints == 3
    assert arm.torque_limits_nm is not None
    assert task.name == "horizontal_cutting"
    assert task.tool_path.shape == (20, 3)
    assert [grasp.name for grasp in grasps] == [
        "short_inline",
        "long_inline",
        "angled_up",
        "angled_down",
    ]
    assert config["output"]["figures_dir"] == "media/figures"


def test_default_config_produces_intended_deterministic_comparison() -> None:
    config = load_demo_config(DEFAULT_CONFIG)
    arm = arm_from_config(config)
    task = task_from_config(config)
    grasps = grasps_from_config(config)

    baseline = plan_baseline(arm, task, grasps)
    force_aware = plan_force_aware(arm, task, grasps)

    assert baseline.success
    assert baseline.diagnostics["torque_feasible"] is False
    assert force_aware.success
    assert force_aware.diagnostics["torque_feasible"] is True
    assert baseline.selected_grasp != force_aware.selected_grasp
    assert baseline.path_q is not None
    assert force_aware.path_q is not None
    assert not np.allclose(baseline.path_q, force_aware.path_q)


@pytest.mark.parametrize("missing_section", ["arm", "task", "grasps", "output"])
def test_config_rejects_missing_required_sections(
    tmp_path: Path,
    missing_section: str,
) -> None:
    config = load_default_as_dict()
    del config[missing_section]

    with pytest.raises(ValueError, match="missing required section"):
        load_demo_config(write_config(tmp_path, config))


def test_config_rejects_invalid_arm_vectors(tmp_path: Path) -> None:
    config = load_default_as_dict()
    config["arm"]["link_lengths_m"] = [1.0, -1.0, 0.7]

    with pytest.raises(ValueError, match="positive"):
        load_demo_config(write_config(tmp_path, config))

    config = load_default_as_dict()
    config["arm"]["torque_limits_nm"] = [18.0, 12.0]

    with pytest.raises(ValueError, match="torque_limits_nm"):
        load_demo_config(write_config(tmp_path, config))

    config = load_default_as_dict()
    config["arm"]["torque_limits_nm"] = [18.0, 0.0, 8.0]

    with pytest.raises(ValueError, match="positive"):
        load_demo_config(write_config(tmp_path, config))


def test_config_rejects_invalid_waypoint_count_and_duplicate_grasps(tmp_path: Path) -> None:
    config = load_default_as_dict()
    config["task"]["num_waypoints"] = 0

    with pytest.raises(ValueError, match="positive integer"):
        load_demo_config(write_config(tmp_path, config))

    config = load_default_as_dict()
    config["grasps"][1]["name"] = config["grasps"][0]["name"]

    with pytest.raises(ValueError, match="unique"):
        load_demo_config(write_config(tmp_path, config))


def test_config_rejects_invalid_task_vectors_and_output_directory(tmp_path: Path) -> None:
    config = load_default_as_dict()
    config["task"]["start_pose"] = [1.2, 0.6]

    with pytest.raises(ValueError, match="start_pose"):
        load_demo_config(write_config(tmp_path, config))

    config = load_default_as_dict()
    config["task"]["desired_wrench"] = [0.0, -10.0]

    with pytest.raises(ValueError, match="desired_wrench"):
        load_demo_config(write_config(tmp_path, config))

    config = load_default_as_dict()
    config["output"]["figures_dir"] = ""

    with pytest.raises(ValueError, match="figures_dir"):
        load_demo_config(write_config(tmp_path, config))
