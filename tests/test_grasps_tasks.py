import numpy as np
import pytest

from force_tool_planning.grasps import Grasp, make_default_grasps, tool_path_to_ee_path
from force_tool_planning.tasks import ToolUseTask, make_horizontal_cutting_task


def test_zero_grasp_transform_returns_same_path() -> None:
    task = make_horizontal_cutting_task(num_waypoints=4)
    zero_grasp = Grasp(name="zero", tool_T_ee=np.zeros(3))

    ee_path = tool_path_to_ee_path(task.tool_path, zero_grasp)

    np.testing.assert_allclose(ee_path, task.tool_path)


def test_different_grasps_produce_different_ee_paths() -> None:
    task = make_horizontal_cutting_task(num_waypoints=3)
    grasps = make_default_grasps()

    short_path = tool_path_to_ee_path(task.tool_path, grasps[0])
    angled_path = tool_path_to_ee_path(task.tool_path, grasps[2])

    assert short_path.shape == task.tool_path.shape
    assert angled_path.shape == task.tool_path.shape
    assert not np.allclose(short_path, angled_path)


def test_default_grasps_have_expected_deterministic_order_and_transforms() -> None:
    grasps = make_default_grasps()

    assert [grasp.name for grasp in grasps] == [
        "short_inline",
        "long_inline",
        "angled_up",
        "angled_down",
    ]
    np.testing.assert_allclose(grasps[0].tool_T_ee, [-0.2, 0.0, 0.0])
    np.testing.assert_allclose(grasps[-1].tool_T_ee, [-0.4, 0.0, -0.6])


def test_grasp_transform_direction_is_tool_to_ee() -> None:
    tool_path = np.array([[1.0, 2.0, np.pi / 2.0]])
    grasp = Grasp(name="offset", tool_T_ee=[-0.2, 0.0, 0.3])

    ee_path = tool_path_to_ee_path(tool_path, grasp)

    np.testing.assert_allclose(
        ee_path[0],
        np.array([1.0, 1.8, np.pi / 2.0 + 0.3]),
        atol=1e-12,
    )


def test_horizontal_cutting_task_has_requested_shape_and_constant_orientation() -> None:
    task = make_horizontal_cutting_task(
        num_waypoints=5,
        start_pose=(0.8, 0.4, 0.2),
        length_m=0.6,
        desired_wrench=(0.0, -8.0, 1.0),
    )

    assert task.tool_path.shape == (5, 3)
    assert task.desired_wrench.shape == (3,)
    np.testing.assert_allclose(task.tool_path[:, 1], 0.4)
    np.testing.assert_allclose(task.tool_path[:, 2], 0.2)
    np.testing.assert_allclose(task.tool_path[[0, -1], 0], np.array([0.8, 1.4]))


def test_grasp_and_task_copy_input_arrays() -> None:
    grasp_pose = np.zeros(3)
    tool_path = np.zeros((2, 3))
    wrench = np.zeros(3)
    grasp = Grasp(name="copy_test", tool_T_ee=grasp_pose)
    task = ToolUseTask(name="copy_test", tool_path=tool_path, desired_wrench=wrench)

    grasp_pose[0] = 1.0
    tool_path[0, 0] = 1.0
    wrench[0] = 1.0

    np.testing.assert_allclose(grasp.tool_T_ee, np.zeros(3))
    np.testing.assert_allclose(task.tool_path, np.zeros((2, 3)))
    np.testing.assert_allclose(task.desired_wrench, np.zeros(3))


def test_task_generation_rejects_invalid_waypoint_count() -> None:
    with pytest.raises(ValueError, match="positive integer"):
        make_horizontal_cutting_task(num_waypoints=0)


def test_grasp_and_task_validation_reject_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="non-empty string"):
        Grasp(name="", tool_T_ee=np.zeros(3))

    with pytest.raises(ValueError, match=r"shape \(3,\)"):
        Grasp(name="bad_pose", tool_T_ee=np.zeros(2))

    with pytest.raises(ValueError, match="tool_path"):
        ToolUseTask(name="bad_path", tool_path=np.zeros((2, 2)), desired_wrench=np.zeros(3))

    with pytest.raises(ValueError, match="desired_wrench"):
        ToolUseTask(name="bad_wrench", tool_path=np.zeros((2, 3)), desired_wrench=np.zeros(2))

    with pytest.raises(ValueError, match="non-negative"):
        make_horizontal_cutting_task(length_m=-0.1)
