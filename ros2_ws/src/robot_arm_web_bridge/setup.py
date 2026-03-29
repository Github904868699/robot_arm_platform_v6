from setuptools import find_packages, setup

package_name = "robot_arm_web_bridge"

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
    description="ROS2 web bridge package for robot arm platform.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ros_control_entry_server = robot_arm_web_bridge.ros_control_entry_server:main",
            "mock_api_server = robot_arm_web_bridge.mock_api_server:main",
        ],
    },
)

