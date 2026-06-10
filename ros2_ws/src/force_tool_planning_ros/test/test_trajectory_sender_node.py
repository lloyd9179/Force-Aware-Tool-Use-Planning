"""Tests for the FollowJointTrajectory sender node configuration."""

from control_msgs.action import FollowJointTrajectory

from force_tool_planning_ros.trajectory_sender_node import ACTION_NAME


def test_sender_targets_force_aware_trajectory_controller_action() -> None:
    assert ACTION_NAME == (
        "/force_aware_trajectory_controller/follow_joint_trajectory"
    )
    assert FollowJointTrajectory.Result.SUCCESSFUL == 0
