"""Launch the Phase 3 live contact-execution RViz demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create a live Phase 3 contact execution launch description."""

    package_share = FindPackageShare("force_tool_planning_ros")
    xacro_path = PathJoinSubstitution(
        [package_share, "urdf", "planar_tool_arm.urdf.xacro"]
    )
    phase3_config = PathJoinSubstitution(
        [package_share, "config", "phase3_contact_execution.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "config", "phase3_contact_execution.rviz"]
    )
    controller_mode = LaunchConfiguration("controller_mode")
    publish_period_s = LaunchConfiguration("publish_period_s")
    robot_description = {"robot_description": Command(["xacro ", xacro_path])}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_mode",
                default_value="force_aware",
                description="Phase 3 controller mode: force_aware or position_only.",
            ),
            DeclareLaunchArgument(
                "publish_period_s",
                default_value="0.05",
                description="Timer period for live Phase 3 sample publication.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="force_tool_planning_ros",
                executable="contact_execution_demo_node",
                parameters=[
                    {
                        "controller_mode": controller_mode,
                        "config_path": phase3_config,
                        "publish_period_s": publish_period_s,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
