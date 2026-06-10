from glob import glob
from os.path import join
from pathlib import Path

from setuptools import find_packages, setup

package_name = "force_tool_planning_ros"
phase1_source = Path(__file__).resolve().parents[3] / "src"
phase1_config = join("..", "..", "..", "configs", "demo_planar_arm.yaml")

setup(
    name=package_name,
    version="0.1.0",
    packages=(
        find_packages(exclude=["test"])
        + find_packages(where=str(phase1_source))
    ),
    package_dir={
        "force_tool_planning": str(phase1_source / "force_tool_planning")
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            join("share", package_name, "config"),
            glob("config/*.rviz") + glob("config/*.yaml") + [phase1_config],
        ),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["numpy", "PyYAML", "setuptools"],
    zip_safe=True,
    maintainer="Lloyd Guan",
    maintainer_email="erwindecoguan@163.com",
    description=(
        "ROS2 visualization and mock-control integration for force-aware "
        "tool-use planning."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_demo_node = "
            "force_tool_planning_ros.joint_state_demo_node:main",
            "result_summary_node = "
            "force_tool_planning_ros.summary_node:main",
            "marker_publisher_node = "
            "force_tool_planning_ros.marker_publisher_node:main",
            "trajectory_sender_node = "
            "force_tool_planning_ros.trajectory_sender_node:main",
        ],
    },
)
