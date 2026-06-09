"""Display the planar arm and attached tool with deterministic joint motion."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the display-only Phase 2 launch description."""

    package_share = FindPackageShare("force_tool_planning_ros")
    xacro_path = PathJoinSubstitution(
        [package_share, "urdf", "planar_tool_arm.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "config", "display.rviz"]
    )
    robot_description = {"robot_description": Command(["xacro ", xacro_path])}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="force_tool_planning_ros",
                executable="joint_state_demo_node",
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
