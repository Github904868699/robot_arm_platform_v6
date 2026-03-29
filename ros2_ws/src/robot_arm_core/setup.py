from setuptools import find_packages, setup

package_name = "robot_arm_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot_arm_platform",
    maintainer_email="dev@example.com",
    description="Core ROS2 python services for robot arm platform.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hardware_control_services = robot_arm_core.hardware_control_services:main",
        ],
    },
)

