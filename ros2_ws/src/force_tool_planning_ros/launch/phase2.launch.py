"""Run the torque-feasible force-aware Phase 2 demo twice."""

from launch import LaunchDescription

from force_tool_planning_ros.demo_launch import build_execution_demo_launch


def generate_launch_description() -> LaunchDescription:
    """Create the default force-aware execution demo."""

    return build_execution_demo_launch(
        planner_name="force_aware",
        execution_selection="force-aware path only; torque feasible",
        ee_to_tool_x=0.20,
        ee_to_tool_y=0.0,
        ee_to_tool_yaw=0.0,
        tool_body_length=0.20,
        repeat_count=2,
    )
