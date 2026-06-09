from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = "force_tool_planning_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (join("share", package_name, "config"), glob("config/*.rviz")),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lloyd Guan",
    maintainer_email="erwindecoguan@163.com",
    description=(
        "ROS2 visualization and mock-control integration for force-aware "
        "tool-use planning."
    ),
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_demo_node = "
            "force_tool_planning_ros.joint_state_demo_node:main",
        ],
    },
)
