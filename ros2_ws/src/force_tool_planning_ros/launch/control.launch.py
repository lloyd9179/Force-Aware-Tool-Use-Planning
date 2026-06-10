"""Start ros2_control mock hardware and the Phase 2 controllers."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the control-only Phase 2 launch description."""

    package_share = FindPackageShare("force_tool_planning_ros")
    xacro_path = PathJoinSubstitution(
        [package_share, "urdf", "planar_tool_arm.urdf.xacro"]
    )
    controller_config = PathJoinSubstitution(
        [package_share, "config", "controllers.yaml"]
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
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controller_config],
                remappings=[("~/robot_description", "/robot_description")],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["force_aware_trajectory_controller"],
                output="screen",
            ),
        ]
    )
