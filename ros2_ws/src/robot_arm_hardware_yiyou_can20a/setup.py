from setuptools import find_packages, setup

package_name = "robot_arm_hardware_yiyou_can20a"

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
    description="Yiyou CAN 2.0A adapter package.",
    license="Apache-2.0",
)
