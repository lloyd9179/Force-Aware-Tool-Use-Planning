"""Run Phase 3 position-only versus force-aware contact comparisons."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from force_tool_planning.config import (
    arm_from_config,
    grasps_from_config,
    load_demo_config,
    task_from_config,
)
from force_tool_planning.contact.contact_metrics import (
    ContactMetricThresholds,
    result_with_contact_metrics,
)
from force_tool_planning.contact.force_estimator import estimate_contact_wrench_2d
from force_tool_planning.contact.contact_model import ContactState, PlanarContactModel
from force_tool_planning.contact.surface import Surface2D
from force_tool_planning.control.force_aware_controller import ForceAwareController
from force_tool_planning.control.position_controller import PositionOnlyController
from force_tool_planning.grasps import Grasp, tool_path_to_ee_path
from force_tool_planning.ik import solve_planar_3link_ik
from force_tool_planning.jacobian import planar_jacobian
from force_tool_planning.kinematics import ArmModel, wrap_to_pi
from force_tool_planning.planner import PlanningResult, plan_force_aware
from force_tool_planning.simulation.contact_execution_stepper import (
    ContactExecutionStepper,
)
from force_tool_planning.simulation.contact_execution_sim import ContactExecutionSimulator
from force_tool_planning.simulation.execution_result import ContactExecutionResult
from force_tool_planning.torque import joint_torques_from_wrench


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PHASE3_CONFIG = REPO_ROOT / "configs" / "phase3_contact_execution.yaml"


@dataclass(frozen=True)
class Phase3Trajectory:
    """Desired tool-tip trajectory and normal-force command for Phase 3."""

    time_s: np.ndarray
    desired_tool_tip_pos_m: np.ndarray
    desired_tool_tip_vel_mps: np.ndarray
    desired_normal_force_n: np.ndarray
    initial_tool_tip_pos_m: np.ndarray


@dataclass(frozen=True)
class Phase1ExecutionReference:
    """Phase 1 force-aware plan used as the Phase 3 torque branch reference."""

    arm: ArmModel
    selected_grasp: Grasp
    planning_result: PlanningResult

    def __post_init__(self) -> None:
        if not self.planning_result.success:
            raise ValueError("Phase 1 reference planning_result must be successful")
        if self.planning_result.path_q is None:
            raise ValueError("Phase 1 reference planning_result.path_q is required")
        if self.planning_result.selected_grasp != self.selected_grasp.name:
            raise ValueError("selected_grasp must match planning_result.selected_grasp")
        if self.arm.torque_limits_nm is None:
            raise ValueError("Phase 1 reference arm must define torque limits")


class Phase1ReferenceTorqueEstimator:
    """Estimate execution torques using Phase 1 IK, Jacobian, and torque math."""

    def __init__(self, reference: Phase1ExecutionReference) -> None:
        self.reference = reference
        self._previous_q_rad: np.ndarray | None = None

    @property
    def current_joint_positions_rad(self) -> np.ndarray | None:
        """Return the latest IK branch selected for execution torque estimates."""

        if self._previous_q_rad is None:
            return None
        return self._previous_q_rad.copy()

    def reset(self) -> None:
        """Clear live execution branch history before replaying a rollout."""

        self._previous_q_rad = None

    def __call__(
        self,
        actual_tool_tip_pos_m: np.ndarray,
        _actual_tool_tip_vel_mps: np.ndarray,
        contact_state: ContactState,
    ) -> np.ndarray:
        """Return joint torques for the current Phase 3 contact state.

        The Phase 3 tool tip is treated as the Phase 1 tool frame origin. The
        selected Phase 1 force-aware grasp maps the current tool pose to an EE
        pose; analytic IK may return multiple branches, so this estimator keeps
        the branch closest to the previous execution sample or the nearest
        Phase 1 reference waypoint.
        """

        contact_normal_force_n = contact_state.normal_force_n
        reference_index = self._nearest_reference_index(actual_tool_tip_pos_m)
        tool_theta_rad = float(self.reference.planning_result.tool_path[reference_index, 2])
        tool_pose = np.array(
            [
                actual_tool_tip_pos_m[0],
                actual_tool_tip_pos_m[1],
                tool_theta_rad,
            ],
            dtype=float,
        )
        ee_pose = tool_path_to_ee_path(
            np.asarray([tool_pose], dtype=float),
            self.reference.selected_grasp,
        )[0]
        ik_candidates = solve_planar_3link_ik(
            self.reference.arm,
            ee_pose,
            include_joint_limit_check=False,
        )
        if not ik_candidates:
            raise ValueError("no IK candidates for Phase 3 executed tool-tip pose")

        seed_q = (
            self._previous_q_rad
            if self._previous_q_rad is not None
            else self.reference.planning_result.path_q[reference_index]
        )
        q_rad = min(
            ik_candidates,
            key=lambda candidate: _wrapped_joint_distance(candidate, seed_q),
        )
        self._previous_q_rad = q_rad.copy()

        contact_point_from_ee_m = actual_tool_tip_pos_m - ee_pose[:2]
        wrench = estimate_contact_wrench_2d(
            normal_force_n=contact_normal_force_n,
            contact_point_m=contact_point_from_ee_m,
        )
        return joint_torques_from_wrench(
            planar_jacobian(self.reference.arm, q_rad),
            wrench,
        )

    def _nearest_reference_index(self, actual_tool_tip_pos_m: np.ndarray) -> int:
        reference_xy = self.reference.planning_result.tool_path[:, :2]
        distances = np.linalg.norm(reference_xy - actual_tool_tip_pos_m, axis=1)
        return int(np.argmin(distances))


@dataclass(frozen=True)
class Phase3ComparisonResult:
    """Results from running both Phase 3 execution controllers."""

    config_path: Path
    position_only: ContactExecutionResult
    force_aware: ContactExecutionResult

    @property
    def results(self) -> tuple[ContactExecutionResult, ContactExecutionResult]:
        """Return results in deterministic report order."""

        return (self.position_only, self.force_aware)

    def by_controller(self) -> dict[str, ContactExecutionResult]:
        """Return results keyed by controller name."""

        return {result.controller_name: result for result in self.results}


def load_phase3_config(config_path: str | Path = DEFAULT_PHASE3_CONFIG) -> dict[str, Any]:
    """Load and validate the Phase 3 contact execution YAML config."""

    path = Path(config_path)
    try:
        with path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML config: {path}") from exc

    config = _require_mapping(loaded, "config")
    for section_name in (
        "demo",
        "arm",
        "tool",
        "task",
        "surface",
        "contact_model",
        "controllers",
        "metrics",
        "output",
    ):
        _require_section(config, section_name)

    build_surface(config)
    build_contact_model(config)
    build_metric_thresholds(config)
    build_controllers(config)
    build_desired_contact_trajectory(config)
    load_torque_limits_nm(config, path)
    build_phase1_execution_reference(config, path)
    return config


def compare_controllers(
    config_path: str | Path = DEFAULT_PHASE3_CONFIG,
) -> Phase3ComparisonResult:
    """Run position-only and force-aware Phase 3 controllers from config."""

    path = Path(config_path)
    config = load_phase3_config(path)
    phase1_reference = build_phase1_execution_reference(config, path)
    trajectory = build_desired_contact_trajectory(config)
    position_controller, force_controller = build_controllers(config)
    thresholds = build_metric_thresholds(config)

    position_simulator = build_contact_execution_simulator(config, path, phase1_reference)
    position_result = position_simulator.run(
        position_controller,
        controller_name="position_only",
        time_s=trajectory.time_s,
        desired_tool_tip_pos_m=trajectory.desired_tool_tip_pos_m,
        desired_tool_tip_vel_mps=trajectory.desired_tool_tip_vel_mps,
        desired_normal_force_n=trajectory.desired_normal_force_n,
        initial_tool_tip_pos_m=trajectory.initial_tool_tip_pos_m,
    )
    force_simulator = build_contact_execution_simulator(config, path, phase1_reference)
    force_result = force_simulator.run(
        force_controller,
        controller_name="force_aware",
        time_s=trajectory.time_s,
        desired_tool_tip_pos_m=trajectory.desired_tool_tip_pos_m,
        desired_tool_tip_vel_mps=trajectory.desired_tool_tip_vel_mps,
        desired_normal_force_n=trajectory.desired_normal_force_n,
        initial_tool_tip_pos_m=trajectory.initial_tool_tip_pos_m,
    )

    return Phase3ComparisonResult(
        config_path=path,
        position_only=result_with_contact_metrics(position_result, thresholds),
        force_aware=result_with_contact_metrics(force_result, thresholds),
    )


def build_contact_execution_stepper(
    config: Mapping[str, Any],
    config_path: str | Path = DEFAULT_PHASE3_CONFIG,
    *,
    controller_mode: str,
) -> ContactExecutionStepper:
    """Build a live pure-Python stepper for one Phase 3 controller mode."""

    trajectory = build_desired_contact_trajectory(config)
    position_controller, force_controller = build_controllers(config)
    if controller_mode == "position_only":
        controller = position_controller
    elif controller_mode == "force_aware":
        controller = force_controller
    else:
        raise ValueError("controller_mode must be 'position_only' or 'force_aware'")

    simulator = build_contact_execution_simulator(config, config_path)
    return ContactExecutionStepper(
        simulator,
        controller,
        controller_name=controller_mode,
        time_s=trajectory.time_s,
        desired_tool_tip_pos_m=trajectory.desired_tool_tip_pos_m,
        desired_tool_tip_vel_mps=trajectory.desired_tool_tip_vel_mps,
        desired_normal_force_n=trajectory.desired_normal_force_n,
        initial_tool_tip_pos_m=trajectory.initial_tool_tip_pos_m,
    )


def build_contact_execution_simulator(
    config: Mapping[str, Any],
    config_path: str | Path = DEFAULT_PHASE3_CONFIG,
    phase1_reference: Phase1ExecutionReference | None = None,
) -> ContactExecutionSimulator:
    """Build the pure-Python Phase 3 simulator from config."""

    reference = phase1_reference or build_phase1_execution_reference(config, config_path)
    return ContactExecutionSimulator(
        surface=build_surface(config),
        contact_model=build_contact_model(config),
        torque_limits_nm=reference.arm.torque_limits_nm,
        torque_estimator=Phase1ReferenceTorqueEstimator(reference),
    )


def build_phase1_execution_reference(
    config: Mapping[str, Any],
    config_path: str | Path = DEFAULT_PHASE3_CONFIG,
) -> Phase1ExecutionReference:
    """Build the Phase 1 force-aware plan used for Phase 3 torque estimates."""

    arm_config = _require_section(config, "arm")
    source_config = _string_value(arm_config, "source_config")
    source_path = _resolve_config_path(config_path, source_config)
    phase1_config = load_demo_config(source_path)
    arm = arm_from_config(phase1_config)
    task = task_from_config(phase1_config)
    grasps = grasps_from_config(phase1_config)
    planning_result = plan_force_aware(arm, task, grasps)
    if not planning_result.success:
        raise ValueError("Phase 1 force-aware planner failed for Phase 3 reference")
    selected_grasp = next(
        grasp for grasp in grasps if grasp.name == planning_result.selected_grasp
    )
    return Phase1ExecutionReference(
        arm=arm,
        selected_grasp=selected_grasp,
        planning_result=planning_result,
    )


def build_surface(config: Mapping[str, Any]) -> Surface2D:
    """Build the configured deterministic 2D surface."""

    task = _require_section(config, "task")
    surface = _require_section(config, "surface")
    return Surface2D(
        surface_type=_string_value(surface, "type"),
        planned_height_m=_float_value(task, "planned_surface_height_m"),
        height_offset_m=_float_value(surface, "height_offset_m", default=0.0),
        amplitude_m=_float_value(surface, "amplitude_m", default=0.0),
        frequency_cycles_per_m=_float_value(
            surface,
            "frequency_cycles_per_m",
            default=0.0,
        ),
    )


def build_contact_model(config: Mapping[str, Any]) -> PlanarContactModel:
    """Build the configured simplified planar contact model."""

    contact = _require_section(config, "contact_model")
    return PlanarContactModel(
        normal_stiffness_n_per_m=_float_value(contact, "normal_stiffness_n_per_m"),
        normal_damping_n_s_per_m=_float_value(contact, "normal_damping_n_s_per_m"),
        friction_coefficient=_float_value(contact, "friction_coefficient"),
        max_penetration_m=_float_value(contact, "max_penetration_m"),
    )


def build_metric_thresholds(config: Mapping[str, Any]) -> ContactMetricThresholds:
    """Build metric thresholds used to classify each controller run."""

    metrics = _require_section(config, "metrics")
    return ContactMetricThresholds(
        contact_loss_force_threshold_n=_float_value(
            metrics,
            "contact_loss_force_threshold_n",
        ),
        excessive_force_threshold_n=_float_value(
            metrics,
            "excessive_force_threshold_n",
        ),
        contact_loss_fraction_threshold=_float_value(
            metrics,
            "contact_loss_fraction_threshold",
        ),
        torque_warning_ratio=_float_value(metrics, "torque_warning_ratio"),
        torque_failure_ratio=_float_value(metrics, "torque_failure_ratio"),
        max_penetration_m=_float_value(metrics, "max_penetration_m"),
    )


def build_controllers(
    config: Mapping[str, Any],
) -> tuple[PositionOnlyController, ForceAwareController]:
    """Build the position-only and force-aware Phase 3 controllers."""

    controllers = _require_section(config, "controllers")
    position = _require_section(controllers, "position_only")
    force = _require_section(controllers, "force_aware")
    return (
        PositionOnlyController(
            kp_task=_float_value(position, "kp_task"),
            kd_task=_float_value(position, "kd_task"),
        ),
        ForceAwareController(
            kp_tangent=_float_value(force, "kp_tangent"),
            kd_tangent=_float_value(force, "kd_tangent"),
            force_gain_mps_per_n=_float_value(force, "force_gain_mps_per_n"),
            max_normal_correction_mps=_float_value(
                force,
                "max_normal_correction_mps",
            ),
            force_deadband_n=_float_value(force, "force_deadband_n"),
        ),
    )


def build_desired_contact_trajectory(config: Mapping[str, Any]) -> Phase3Trajectory:
    """Build the deterministic planned tool-tip trajectory from config."""

    task = _require_section(config, "task")
    contact = _require_section(config, "contact_model")
    duration_s = _positive_float_value(task, "duration_s")
    dt_s = _positive_float_value(task, "dt_s")
    start_m = _float_value(task, "tangential_start_m")
    end_m = _float_value(task, "tangential_end_m")
    speed_mps = _positive_float_value(task, "desired_tangential_speed_mps")
    planned_height_m = _float_value(task, "planned_surface_height_m")
    desired_force_n = _positive_float_value(task, "desired_normal_force_n")
    stiffness_n_per_m = _positive_float_value(contact, "normal_stiffness_n_per_m")
    surface = build_surface(config)

    time_s = _time_samples(duration_s, dt_s)
    direction = 1.0 if end_m >= start_m else -1.0
    raw_x_m = start_m + direction * speed_mps * time_s
    x_m = np.clip(raw_x_m, min(start_m, end_m), max(start_m, end_m))
    x_velocity_mps = np.where(
        np.isclose(x_m, end_m),
        0.0,
        direction * speed_mps,
    )
    desired_y_m = planned_height_m - desired_force_n / stiffness_n_per_m
    desired_pos_m = np.column_stack((x_m, np.full_like(x_m, desired_y_m)))
    desired_vel_mps = np.column_stack((x_velocity_mps, np.zeros_like(x_m)))
    desired_force = np.full_like(time_s, desired_force_n)
    initial_y_m = surface.height(start_m) - desired_force_n / stiffness_n_per_m

    return Phase3Trajectory(
        time_s=time_s,
        desired_tool_tip_pos_m=desired_pos_m,
        desired_tool_tip_vel_mps=desired_vel_mps,
        desired_normal_force_n=desired_force,
        initial_tool_tip_pos_m=np.array([start_m, initial_y_m], dtype=float),
    )


def load_torque_limits_nm(
    config: Mapping[str, Any],
    config_path: str | Path = DEFAULT_PHASE3_CONFIG,
) -> np.ndarray:
    """Return Phase 3 torque limits, normally reused from Phase 1 config."""

    arm = _require_section(config, "arm")
    if "torque_limits_nm" in arm:
        return _positive_vector(arm["torque_limits_nm"], "arm.torque_limits_nm")

    source_config = _string_value(arm, "source_config")
    source_path = _resolve_config_path(config_path, source_config)
    phase1_config = load_demo_config(source_path)
    phase1_arm = arm_from_config(phase1_config)
    if phase1_arm.torque_limits_nm is None:
        raise ValueError("Phase 1 arm config must define torque_limits_nm")
    return phase1_arm.torque_limits_nm.copy()


def format_comparison_summary(comparison: Phase3ComparisonResult) -> str:
    """Return the concise numeric summary used by Phase 3 scripts."""

    sections: list[str] = []
    for result in comparison.results:
        reasons = ", ".join(result.failure_reasons) if result.failure_reasons else "none"
        sections.append(
            "\n".join(
                [
                    f"Controller: {result.controller_name}",
                    f"  Force RMSE: {result.metrics['force_rmse_n']:.6f} N",
                    (
                        "  Contact loss fraction: "
                        f"{result.metrics['contact_loss_fraction']:.6f}"
                    ),
                    f"  Max penetration: {result.metrics['max_penetration_m']:.6f} m",
                    f"  Max torque ratio: {result.metrics['max_torque_ratio']:.6f}",
                    f"  Success: {'yes' if result.metrics['success'] else 'no'}",
                    f"  Failure reasons: {reasons}",
                ]
            )
        )
    return "\n\n".join(sections)


def _time_samples(duration_s: float, dt_s: float) -> np.ndarray:
    step_count = int(np.floor(duration_s / dt_s))
    time_s = np.arange(step_count + 1, dtype=float) * dt_s
    if time_s[-1] < duration_s:
        time_s = np.append(time_s, duration_s)
    return time_s


def _wrapped_joint_distance(first_q_rad: np.ndarray, second_q_rad: np.ndarray) -> float:
    delta_q_rad = wrap_to_pi(first_q_rad - second_q_rad)
    return float(np.dot(delta_q_rad, delta_q_rad))


def _resolve_config_path(config_path: str | Path, referenced_path: str) -> Path:
    path = Path(referenced_path)
    if path.is_absolute():
        return path
    base_path = Path(config_path).resolve().parent
    candidate = base_path / path
    if candidate.exists():
        return candidate
    sibling_candidate = base_path / path.name
    if sibling_candidate.exists():
        return sibling_candidate
    return REPO_ROOT / path


def _require_mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _require_section(config: Mapping[str, Any], name: str) -> dict[str, Any]:
    if name not in config:
        raise ValueError(f"missing required section: {name}")
    return _require_mapping(config[name], name)


def _string_value(section: Mapping[str, Any], key: str) -> str:
    if key not in section:
        raise ValueError(f"missing required field: {key}")
    value = section[key]
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _float_value(
    section: Mapping[str, Any],
    key: str,
    *,
    default: float | None = None,
) -> float:
    if key not in section:
        if default is not None:
            return default
        raise ValueError(f"missing required field: {key}")
    try:
        value = float(section[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be numeric") from exc
    if not np.isfinite(value):
        raise ValueError(f"{key} must be finite")
    return value


def _positive_float_value(section: Mapping[str, Any], key: str) -> float:
    value = _float_value(section, key)
    if value <= 0.0:
        raise ValueError(f"{key} must be positive")
    return value


def _positive_vector(value: object, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.ndim != 1 or vector.size == 0:
        raise ValueError(f"{name} must be a non-empty 1D vector")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain only finite values")
    if np.any(vector <= 0.0):
        raise ValueError(f"{name} must contain only positive values")
    return vector.copy()
