"""Layered planning for the Phase 1 force-aware tool-use demo."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from force_tool_planning.grasps import Grasp, tool_path_to_ee_path
from force_tool_planning.ik import solve_planar_3link_ik
from force_tool_planning.jacobian import planar_jacobian
from force_tool_planning.kinematics import ArmModel, within_joint_limits, wrap_to_pi
from force_tool_planning.tasks import ToolUseTask
from force_tool_planning.torque import (
    TorqueCheckResult,
    check_torque_limits,
    joint_torques_from_wrench,
)

_COST_ATOL = 1e-12


@dataclass(frozen=True)
class IKCandidate:
    """One IK branch and its geometric and torque-feasibility diagnostics."""

    waypoint_index: int
    grasp_name: str
    q: np.ndarray
    ee_pose: np.ndarray
    tool_pose: np.ndarray
    joint_limit_feasible: bool
    torque_check: TorqueCheckResult | None


@dataclass(frozen=True)
class PlanningResult:
    """Structured result from a baseline or force-aware planning request."""

    planner_name: str
    success: bool
    selected_grasp: str | None
    path_q: np.ndarray | None
    tool_path: np.ndarray
    ee_path: np.ndarray | None
    desired_wrench: np.ndarray
    failure_reason: str | None
    total_candidates: int
    joint_limit_feasible_candidates: int
    torque_feasible_candidates: int
    rejected_by_joint_limits: list[IKCandidate]
    rejected_by_torque: list[IKCandidate]
    selected_candidates: list[IKCandidate]
    max_torque_ratio: float | None
    diagnostics: dict[str, object]


def _torque_check_for_q(
    arm: ArmModel,
    q: np.ndarray,
    desired_wrench: np.ndarray,
) -> TorqueCheckResult:
    if arm.torque_limits_nm is None:
        raise ValueError("arm.torque_limits_nm is required for torque checks")
    torque_nm = joint_torques_from_wrench(planar_jacobian(arm, q), desired_wrench)
    return check_torque_limits(torque_nm, arm.torque_limits_nm)


def generate_candidates_for_grasp(
    arm: ArmModel,
    task: ToolUseTask,
    grasp: Grasp,
    *,
    check_torque: bool,
) -> list[list[IKCandidate]]:
    """Return all analytic IK candidates at every waypoint for one grasp.

    Candidate objects retain joint-limit and torque-check outcomes. This
    function does not filter rejected candidates from the returned layers.
    """

    if check_torque and arm.torque_limits_nm is None:
        raise ValueError("arm.torque_limits_nm is required when check_torque=True")

    ee_path = tool_path_to_ee_path(task.tool_path, grasp)
    layers: list[list[IKCandidate]] = []
    for waypoint_index, (tool_pose, ee_pose) in enumerate(zip(task.tool_path, ee_path)):
        layer: list[IKCandidate] = []
        for q in solve_planar_3link_ik(arm, ee_pose):
            torque_check = (
                _torque_check_for_q(arm, q, task.desired_wrench)
                if check_torque
                else None
            )
            layer.append(
                IKCandidate(
                    waypoint_index=waypoint_index,
                    grasp_name=grasp.name,
                    q=q.copy(),
                    ee_pose=ee_pose.copy(),
                    tool_pose=tool_pose.copy(),
                    joint_limit_feasible=within_joint_limits(arm, q),
                    torque_check=torque_check,
                )
            )
        layers.append(layer)
    return layers


def _transition_cost(previous: IKCandidate, current: IKCandidate) -> float:
    delta_q = wrap_to_pi(current.q - previous.q)
    return float(np.dot(delta_q, delta_q))


def _path_smoothness_cost(path: list[IKCandidate]) -> float:
    return float(
        sum(
            _transition_cost(previous, current)
            for previous, current in zip(path, path[1:])
        )
    )


def select_smoothest_path(
    layers: list[list[IKCandidate]],
) -> list[IKCandidate] | None:
    """Select one candidate per waypoint minimizing squared wrapped joint motion."""

    if not layers or any(not layer for layer in layers):
        return None

    costs = np.zeros(len(layers[0]), dtype=float)
    predecessor_layers: list[list[int]] = []

    for previous_layer, current_layer in zip(layers, layers[1:]):
        current_costs = np.full(len(current_layer), np.inf, dtype=float)
        current_predecessors = [-1] * len(current_layer)
        for current_index, current in enumerate(current_layer):
            for previous_index, previous in enumerate(previous_layer):
                candidate_cost = costs[previous_index] + _transition_cost(previous, current)
                if candidate_cost < current_costs[current_index] - _COST_ATOL:
                    current_costs[current_index] = candidate_cost
                    current_predecessors[current_index] = previous_index
        costs = current_costs
        predecessor_layers.append(current_predecessors)

    final_index = 0
    for candidate_index in range(1, len(costs)):
        if costs[candidate_index] < costs[final_index] - _COST_ATOL:
            final_index = candidate_index

    selected_indices = [final_index]
    for predecessors in reversed(predecessor_layers):
        selected_indices.append(predecessors[selected_indices[-1]])
    selected_indices.reverse()
    return [layer[index] for layer, index in zip(layers, selected_indices)]


def _flatten_layers(layers: list[list[IKCandidate]]) -> list[IKCandidate]:
    return [candidate for layer in layers for candidate in layer]


def _filter_layers(
    layers: list[list[IKCandidate]],
    *,
    require_torque_feasible: bool,
) -> list[list[IKCandidate]]:
    filtered_layers: list[list[IKCandidate]] = []
    for layer in layers:
        filtered_layers.append(
            [
                candidate
                for candidate in layer
                if candidate.joint_limit_feasible
                and (
                    not require_torque_feasible
                    or (
                        candidate.torque_check is not None
                        and candidate.torque_check.feasible
                    )
                )
            ]
        )
    return filtered_layers


def _max_torque_ratio(path: list[IKCandidate]) -> float | None:
    torque_checks = [
        candidate.torque_check
        for candidate in path
        if candidate.torque_check is not None
    ]
    if not torque_checks:
        return None
    return max(
        float(np.max(np.abs(check.torque_nm) / check.torque_limits_nm))
        for check in torque_checks
    )


def _selected_path_diagnostics(
    selected_path: list[IKCandidate],
    smoothness_cost: float,
) -> dict[str, object]:
    torque_checks = [
        candidate.torque_check
        for candidate in selected_path
        if candidate.torque_check is not None
    ]
    torque_feasible: bool | None
    if len(torque_checks) != len(selected_path):
        torque_feasible = None
    else:
        torque_feasible = all(check.feasible for check in torque_checks)

    violating_joint_indices = sorted(
        {
            int(index)
            for check in torque_checks
            for index in check.violating_joint_indices
        }
    )
    return {
        "path_smoothness_cost": smoothness_cost,
        "torque_feasible": torque_feasible,
        "violating_joint_indices": violating_joint_indices,
    }


def _failure_reason(
    *,
    total_candidates: int,
    joint_limit_feasible_candidates: int,
    torque_feasible_candidates: int,
    require_torque_feasible: bool,
) -> str:
    if total_candidates == 0:
        return "no_ik_candidates"
    if joint_limit_feasible_candidates == 0:
        return "no_joint_limit_feasible_candidates"
    if require_torque_feasible and torque_feasible_candidates == 0:
        return "no_torque_feasible_candidates"
    return "no_complete_layered_path"


def _make_failure_result(
    *,
    planner_name: str,
    task: ToolUseTask,
    all_candidates: list[IKCandidate],
    per_grasp_diagnostics: dict[str, object],
    require_torque_feasible: bool,
) -> PlanningResult:
    rejected_by_joint_limits = [
        candidate for candidate in all_candidates if not candidate.joint_limit_feasible
    ]
    rejected_by_torque = (
        [
            candidate
            for candidate in all_candidates
            if candidate.joint_limit_feasible
            and candidate.torque_check is not None
            and not candidate.torque_check.feasible
        ]
        if require_torque_feasible
        else []
    )
    joint_limit_feasible_candidates = sum(
        candidate.joint_limit_feasible for candidate in all_candidates
    )
    torque_feasible_candidates = sum(
        candidate.joint_limit_feasible
        and candidate.torque_check is not None
        and candidate.torque_check.feasible
        for candidate in all_candidates
    )
    failure_reason = _failure_reason(
        total_candidates=len(all_candidates),
        joint_limit_feasible_candidates=joint_limit_feasible_candidates,
        torque_feasible_candidates=torque_feasible_candidates,
        require_torque_feasible=require_torque_feasible,
    )
    return PlanningResult(
        planner_name=planner_name,
        success=False,
        selected_grasp=None,
        path_q=None,
        tool_path=task.tool_path.copy(),
        ee_path=None,
        desired_wrench=task.desired_wrench.copy(),
        failure_reason=failure_reason,
        total_candidates=len(all_candidates),
        joint_limit_feasible_candidates=joint_limit_feasible_candidates,
        torque_feasible_candidates=torque_feasible_candidates,
        rejected_by_joint_limits=rejected_by_joint_limits,
        rejected_by_torque=rejected_by_torque,
        selected_candidates=[],
        max_torque_ratio=None,
        diagnostics={"per_grasp": per_grasp_diagnostics},
    )


def _plan_across_grasps(
    arm: ArmModel,
    task: ToolUseTask,
    grasps: list[Grasp],
    *,
    planner_name: str,
    require_torque_feasible: bool,
) -> PlanningResult:
    if require_torque_feasible and arm.torque_limits_nm is None:
        raise ValueError("arm.torque_limits_nm is required for force-aware planning")

    check_torque = arm.torque_limits_nm is not None
    all_candidates: list[IKCandidate] = []
    per_grasp_diagnostics: dict[str, object] = {}
    best_grasp: Grasp | None = None
    best_path: list[IKCandidate] | None = None
    best_cost = np.inf

    for grasp in grasps:
        layers = generate_candidates_for_grasp(
            arm,
            task,
            grasp,
            check_torque=check_torque,
        )
        candidates = _flatten_layers(layers)
        all_candidates.extend(candidates)
        filtered_layers = _filter_layers(
            layers,
            require_torque_feasible=require_torque_feasible,
        )
        path = select_smoothest_path(filtered_layers)
        path_cost = _path_smoothness_cost(path) if path is not None else None
        per_grasp_diagnostics[grasp.name] = {
            "layer_candidate_counts": [len(layer) for layer in layers],
            "accepted_layer_candidate_counts": [len(layer) for layer in filtered_layers],
            "complete_path_found": path is not None,
            "path_smoothness_cost": path_cost,
        }
        if path is not None and path_cost is not None and path_cost < best_cost - _COST_ATOL:
            best_grasp = grasp
            best_path = path
            best_cost = path_cost

    if best_grasp is None or best_path is None:
        return _make_failure_result(
            planner_name=planner_name,
            task=task,
            all_candidates=all_candidates,
            per_grasp_diagnostics=per_grasp_diagnostics,
            require_torque_feasible=require_torque_feasible,
        )

    rejected_by_joint_limits = [
        candidate for candidate in all_candidates if not candidate.joint_limit_feasible
    ]
    rejected_by_torque = (
        [
            candidate
            for candidate in all_candidates
            if candidate.joint_limit_feasible
            and candidate.torque_check is not None
            and not candidate.torque_check.feasible
        ]
        if require_torque_feasible
        else []
    )
    joint_limit_feasible_candidates = sum(
        candidate.joint_limit_feasible for candidate in all_candidates
    )
    torque_feasible_candidates = sum(
        candidate.joint_limit_feasible
        and candidate.torque_check is not None
        and candidate.torque_check.feasible
        for candidate in all_candidates
    )
    diagnostics = _selected_path_diagnostics(best_path, best_cost)
    diagnostics["per_grasp"] = per_grasp_diagnostics

    return PlanningResult(
        planner_name=planner_name,
        success=True,
        selected_grasp=best_grasp.name,
        path_q=np.asarray([candidate.q for candidate in best_path], dtype=float),
        tool_path=task.tool_path.copy(),
        ee_path=tool_path_to_ee_path(task.tool_path, best_grasp),
        desired_wrench=task.desired_wrench.copy(),
        failure_reason=None,
        total_candidates=len(all_candidates),
        joint_limit_feasible_candidates=joint_limit_feasible_candidates,
        torque_feasible_candidates=torque_feasible_candidates,
        rejected_by_joint_limits=rejected_by_joint_limits,
        rejected_by_torque=rejected_by_torque,
        selected_candidates=best_path,
        max_torque_ratio=_max_torque_ratio(best_path),
        diagnostics=diagnostics,
    )


def plan_baseline(
    arm: ArmModel,
    task: ToolUseTask,
    grasps: list[Grasp],
) -> PlanningResult:
    """Plan using analytic IK and joint limits, without torque filtering."""

    return _plan_across_grasps(
        arm,
        task,
        grasps,
        planner_name="baseline",
        require_torque_feasible=False,
    )


def plan_force_aware(
    arm: ArmModel,
    task: ToolUseTask,
    grasps: list[Grasp],
) -> PlanningResult:
    """Plan using analytic IK, joint limits, and torque-limit filtering."""

    return _plan_across_grasps(
        arm,
        task,
        grasps,
        planner_name="force_aware",
        require_torque_feasible=True,
    )
