"""Build complete Phase 2 execution demos without duplicating launch wiring."""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def build_execution_demo_launch(
    *,
    planner_name: str,
    execution_selection: str,
    ee_to_tool_x: float,
    ee_to_tool_y: float,
    ee_to_tool_yaw: float,
    tool_body_length: float,
    repeat_count: int = 2,
) -> LaunchDescription:
    """Build a complete visualization and mock-control execution demo."""

    package_share = FindPackageShare("force_tool_planning_ros")
    xacro_path = PathJoinSubstitution(
        [package_share, "urdf", "planar_tool_arm.urdf.xacro"]
    )
    controller_config = PathJoinSubstitution(
        [package_share, "config", "controllers.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "config", "display.rviz"]
    )
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_path,
                f" ee_to_tool_x:={ee_to_tool_x}",
                f" ee_to_tool_y:={ee_to_tool_y}",
                f" ee_to_tool_yaw:={ee_to_tool_yaw}",
                f" tool_body_length:={tool_body_length}",
            ]
        )
    }

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["force_aware_trajectory_controller"],
        output="screen",
    )
    trajectory_sender = Node(
        package="force_tool_planning_ros",
        executable="trajectory_sender_node",
        parameters=[
            {
                "planner_name": planner_name,
                "repeat_count": repeat_count,
            }
        ],
        output="screen",
    )

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
            trajectory_controller_spawner,
            Node(
                package="force_tool_planning_ros",
                executable="result_summary_node",
                parameters=[{"execution_selection": execution_selection}],
                output="screen",
            ),
            Node(
                package="force_tool_planning_ros",
                executable="marker_publisher_node",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=trajectory_controller_spawner,
                    on_exit=[trajectory_sender],
                )
            ),
        ]
    )
