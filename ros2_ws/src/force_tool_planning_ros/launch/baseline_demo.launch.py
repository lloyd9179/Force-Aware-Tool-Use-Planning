"""Run the torque-infeasible baseline path twice for visual comparison."""

from launch import LaunchDescription

from force_tool_planning_ros.demo_launch import build_execution_demo_launch


def generate_launch_description() -> LaunchDescription:
    """Create the explicitly labeled baseline comparison demo."""

    return build_execution_demo_launch(
        planner_name="baseline",
        execution_selection=(
            "baseline comparison path; torque-infeasible visualization only"
        ),
        ee_to_tool_x=0.33013424596387136,
        ee_to_tool_y=0.22585698935801415,
        ee_to_tool_yaw=0.6,
        tool_body_length=0.40,
        repeat_count=2,
    )
