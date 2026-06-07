import numpy as np
import pytest

from force_tool_planning.grasps import Grasp, make_default_grasps
from force_tool_planning.kinematics import ArmModel, forward_kinematics
from force_tool_planning.planner import (
    IKCandidate,
    generate_candidates_for_grasp,
    plan_baseline,
    plan_force_aware,
    select_smoothest_path,
)
from force_tool_planning.tasks import ToolUseTask, make_horizontal_cutting_task


def make_candidate(waypoint_index: int, q: list[float], *, name: str = "test") -> IKCandidate:
    return IKCandidate(
        waypoint_index=waypoint_index,
        grasp_name=name,
        q=np.asarray(q, dtype=float),
        ee_pose=np.zeros(3),
        tool_pose=np.zeros(3),
        joint_limit_feasible=True,
        torque_check=None,
    )


def make_demo_arm() -> ArmModel:
    return ArmModel(
        link_lengths_m=[1.0, 1.0, 0.7],
        joint_limits_rad=[
            [-np.pi, np.pi],
            [-2.6179938779914944, 2.6179938779914944],
            [-2.6179938779914944, 2.6179938779914944],
        ],
        torque_limits_nm=[18.0, 12.0, 8.0],
    )


def test_candidate_generation_preserves_rejected_candidates() -> None:
    unrestricted_arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])
    target_pose = forward_kinematics(unrestricted_arm, [0.3, 1.0, -0.4])
    arm = ArmModel(
        link_lengths_m=[1.0, 1.0, 0.5],
        joint_limits_rad=[
            [0.25, 0.35],
            [0.95, 1.05],
            [-0.45, -0.35],
        ],
        torque_limits_nm=[100.0, 100.0, 100.0],
    )
    task = ToolUseTask(
        name="single_pose",
        tool_path=np.asarray([target_pose]),
        desired_wrench=np.array([0.0, -1.0, 0.0]),
    )

    layers = generate_candidates_for_grasp(
        arm,
        task,
        Grasp(name="zero", tool_T_ee=np.zeros(3)),
        check_torque=True,
    )

    assert len(layers) == 1
    assert len(layers[0]) == 2
    assert sum(candidate.joint_limit_feasible for candidate in layers[0]) == 1
    assert all(candidate.torque_check is not None for candidate in layers[0])


def test_select_smoothest_path_returns_only_candidates_in_singleton_layers() -> None:
    expected = [
        make_candidate(0, [0.0, 0.0, 0.0]),
        make_candidate(1, [0.1, 0.0, 0.0]),
    ]

    assert select_smoothest_path([[expected[0]], [expected[1]]]) == expected


def test_select_smoothest_path_returns_none_if_any_layer_is_empty() -> None:
    assert select_smoothest_path([[make_candidate(0, [0.0, 0.0, 0.0])], []]) is None
    assert select_smoothest_path([]) is None


def test_select_smoothest_path_chooses_lower_joint_motion() -> None:
    smooth = [
        make_candidate(0, [0.0, 0.0, 0.0], name="smooth"),
        make_candidate(1, [0.1, 0.0, 0.0], name="smooth"),
        make_candidate(2, [0.2, 0.0, 0.0], name="smooth"),
    ]
    rough = [
        make_candidate(0, [2.0, 0.0, 0.0], name="rough"),
        make_candidate(1, [1.7, 0.0, 0.0], name="rough"),
        make_candidate(2, [1.4, 0.0, 0.0], name="rough"),
    ]
    layers = [[smooth[index], rough[index]] for index in range(3)]

    selected = select_smoothest_path(layers)

    assert selected == smooth


def test_select_smoothest_path_preserves_earlier_order_on_tie() -> None:
    earlier = make_candidate(0, [-0.5, 0.0, 0.0], name="earlier")
    later = make_candidate(0, [0.5, 0.0, 0.0], name="later")
    final = make_candidate(1, [0.0, 0.0, 0.0])

    selected = select_smoothest_path([[earlier, later], [final]])

    assert selected == [earlier, final]


def test_select_smoothest_path_wraps_joint_motion_across_pi_boundary() -> None:
    start = make_candidate(0, [np.pi - 0.1, 0.0, 0.0])
    wrapped_nearby = make_candidate(1, [-np.pi + 0.1, 0.0, 0.0], name="wrapped")
    unwrapped_farther = make_candidate(1, [2.0, 0.0, 0.0], name="farther")

    selected = select_smoothest_path([[start], [wrapped_nearby, unwrapped_farther]])

    assert selected == [start, wrapped_nearby]


def test_baseline_and_force_aware_produce_intended_deterministic_comparison() -> None:
    arm = make_demo_arm()
    task = make_horizontal_cutting_task()
    grasps = make_default_grasps()

    baseline = plan_baseline(arm, task, grasps)
    force_aware = plan_force_aware(arm, task, grasps)

    assert baseline.success
    assert baseline.diagnostics["torque_feasible"] is False
    assert baseline.max_torque_ratio is not None and baseline.max_torque_ratio > 1.0
    assert force_aware.success
    assert force_aware.diagnostics["torque_feasible"] is True
    assert force_aware.max_torque_ratio is not None and force_aware.max_torque_ratio <= 1.0
    assert baseline.selected_grasp != force_aware.selected_grasp
    assert baseline.path_q is not None
    assert force_aware.path_q is not None
    assert not np.allclose(baseline.path_q, force_aware.path_q)
    assert baseline.rejected_by_torque == []
    assert force_aware.rejected_by_torque


def test_force_aware_returns_failure_when_no_torque_feasible_candidates_exist() -> None:
    arm = ArmModel(
        link_lengths_m=[1.0, 1.0, 0.5],
        joint_limits_rad=[[-np.pi, np.pi]] * 3,
        torque_limits_nm=[0.01, 0.01, 0.01],
    )
    task = ToolUseTask(
        name="infeasible_wrench",
        tool_path=np.array([[1.5, 0.5, 0.0]]),
        desired_wrench=np.array([0.0, -10.0, 0.0]),
    )
    grasp = Grasp(name="zero", tool_T_ee=np.zeros(3))

    baseline = plan_baseline(arm, task, [grasp])
    force_aware = plan_force_aware(arm, task, [grasp])

    assert baseline.success
    assert baseline.diagnostics["torque_feasible"] is False
    assert not force_aware.success
    assert force_aware.failure_reason == "no_torque_feasible_candidates"
    assert force_aware.selected_candidates == []
    assert force_aware.rejected_by_torque


def test_force_aware_requires_torque_limits() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])
    task = ToolUseTask(
        name="missing_limits",
        tool_path=np.array([[1.5, 0.5, 0.0]]),
        desired_wrench=np.array([0.0, -1.0, 0.0]),
    )

    with pytest.raises(ValueError, match="torque_limits_nm"):
        plan_force_aware(arm, task, [Grasp(name="zero", tool_T_ee=np.zeros(3))])


def test_planner_returns_structured_no_ik_failure() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])
    task = ToolUseTask(
        name="unreachable",
        tool_path=np.array([[4.0, 0.0, 0.0]]),
        desired_wrench=np.zeros(3),
    )

    result = plan_baseline(arm, task, [Grasp(name="zero", tool_T_ee=np.zeros(3))])

    assert not result.success
    assert result.failure_reason == "no_ik_candidates"
    assert result.path_q is None
    assert result.selected_candidates == []


def test_planner_returns_structured_joint_limit_failure() -> None:
    unrestricted_arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])
    target_pose = forward_kinematics(unrestricted_arm, [0.3, 1.0, -0.4])
    arm = ArmModel(
        link_lengths_m=[1.0, 1.0, 0.5],
        joint_limits_rad=[[-0.01, 0.01]] * 3,
    )
    task = ToolUseTask(
        name="outside_limits",
        tool_path=np.asarray([target_pose]),
        desired_wrench=np.zeros(3),
    )

    result = plan_baseline(arm, task, [Grasp(name="zero", tool_T_ee=np.zeros(3))])

    assert not result.success
    assert result.failure_reason == "no_joint_limit_feasible_candidates"
    assert len(result.rejected_by_joint_limits) == 2


def test_planner_returns_no_complete_path_when_one_layer_has_no_ik() -> None:
    arm = ArmModel(link_lengths_m=[1.0, 1.0, 0.5])
    task = ToolUseTask(
        name="broken_path",
        tool_path=np.array([[1.5, 0.5, 0.0], [4.0, 0.0, 0.0]]),
        desired_wrench=np.zeros(3),
    )

    result = plan_baseline(arm, task, [Grasp(name="zero", tool_T_ee=np.zeros(3))])

    assert not result.success
    assert result.total_candidates > 0
    assert result.joint_limit_feasible_candidates > 0
    assert result.failure_reason == "no_complete_layered_path"
