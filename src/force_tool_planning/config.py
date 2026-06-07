"""YAML configuration loading for the deterministic Phase 1 demo."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml

from force_tool_planning.grasps import Grasp
from force_tool_planning.kinematics import ArmModel
from force_tool_planning.tasks import ToolUseTask, make_horizontal_cutting_task


def _require_mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _require_section(config: dict[str, Any], name: str) -> dict[str, Any]:
    if name not in config:
        raise ValueError(f"missing required section: {name}")
    return _require_mapping(config[name], name)


def _require_keys(section: dict[str, Any], section_name: str, keys: tuple[str, ...]) -> None:
    missing = [key for key in keys if key not in section]
    if missing:
        raise ValueError(
            f"{section_name} is missing required fields: {', '.join(missing)}"
        )


def _as_finite_array(value: object, name: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain numeric values") from exc
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def arm_from_config(config: dict) -> ArmModel:
    """Build and validate the deterministic 3-link planar arm from config."""

    root = _require_mapping(config, "config")
    arm_config = _require_section(root, "arm")
    _require_keys(
        arm_config,
        "arm",
        ("link_lengths_m", "joint_limits_rad", "torque_limits_nm"),
    )

    link_lengths_m = _as_finite_array(arm_config["link_lengths_m"], "link_lengths_m")
    joint_limits_rad = _as_finite_array(arm_config["joint_limits_rad"], "joint_limits_rad")
    torque_limits_nm = _as_finite_array(arm_config["torque_limits_nm"], "torque_limits_nm")
    arm = ArmModel(
        link_lengths_m=link_lengths_m,
        joint_limits_rad=joint_limits_rad,
        torque_limits_nm=torque_limits_nm,
    )
    if arm.n_joints != 3:
        raise ValueError("Phase 1 demo arm must have exactly 3 joints")
    return arm


def task_from_config(config: dict) -> ToolUseTask:
    """Build and validate the deterministic horizontal tool-use task from config."""

    root = _require_mapping(config, "config")
    task_config = _require_section(root, "task")
    _require_keys(
        task_config,
        "task",
        ("name", "num_waypoints", "start_pose", "length_m", "desired_wrench"),
    )

    name = task_config["name"]
    if not isinstance(name, str) or not name.strip():
        raise ValueError("task.name must be a non-empty string")
    num_waypoints = task_config["num_waypoints"]
    if isinstance(num_waypoints, bool) or not isinstance(num_waypoints, int):
        raise ValueError("task.num_waypoints must be a positive integer")

    start_pose = _as_finite_array(task_config["start_pose"], "task.start_pose")
    desired_wrench = _as_finite_array(
        task_config["desired_wrench"],
        "task.desired_wrench",
    )
    if start_pose.shape != (3,):
        raise ValueError("task.start_pose must have shape (3,)")
    if desired_wrench.shape != (3,):
        raise ValueError("task.desired_wrench must have shape (3,)")

    try:
        length_m = float(task_config["length_m"])
    except (TypeError, ValueError) as exc:
        raise ValueError("task.length_m must be numeric") from exc

    generated_task = make_horizontal_cutting_task(
        num_waypoints=num_waypoints,
        start_pose=tuple(float(value) for value in start_pose),
        length_m=length_m,
        desired_wrench=tuple(float(value) for value in desired_wrench),
    )
    return ToolUseTask(
        name=name,
        tool_path=generated_task.tool_path,
        desired_wrench=generated_task.desired_wrench,
    )


def grasps_from_config(config: dict) -> list[Grasp]:
    """Build and validate the ordered deterministic grasp candidates from config."""

    root = _require_mapping(config, "config")
    if "grasps" not in root:
        raise ValueError("missing required section: grasps")
    grasp_entries = root["grasps"]
    if not isinstance(grasp_entries, list) or not grasp_entries:
        raise ValueError("grasps must be a non-empty list")

    grasps: list[Grasp] = []
    seen_names: set[str] = set()
    for index, entry in enumerate(grasp_entries):
        grasp_config = _require_mapping(entry, f"grasps[{index}]")
        _require_keys(grasp_config, f"grasps[{index}]", ("name", "tool_T_ee"))
        grasp = Grasp(
            name=grasp_config["name"],
            tool_T_ee=_as_finite_array(
                grasp_config["tool_T_ee"],
                f"grasps[{index}].tool_T_ee",
            ),
        )
        if grasp.name in seen_names:
            raise ValueError(f"grasp names must be unique: {grasp.name}")
        seen_names.add(grasp.name)
        grasps.append(grasp)
    return grasps


def _validate_output_config(config: dict[str, Any]) -> None:
    output_config = _require_section(config, "output")
    _require_keys(output_config, "output", ("figures_dir",))
    figures_dir = output_config["figures_dir"]
    if not isinstance(figures_dir, str) or not figures_dir.strip():
        raise ValueError("output.figures_dir must be a non-empty string")


def load_demo_config(path: str | Path) -> dict:
    """Load a YAML demo config and return a validated dictionary."""

    config_path = Path(path)
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML config: {config_path}") from exc

    config = _require_mapping(loaded, "config")
    arm_from_config(config)
    task_from_config(config)
    grasps_from_config(config)
    _validate_output_config(config)
    return config
