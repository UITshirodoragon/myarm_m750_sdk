from glob import glob

from setuptools import find_packages, setup

package_name = "myarm_m750_driver"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nguyen Hoang Dang Khoa",
    maintainer_email="khoa@example.invalid",
    description="Lifecycle and FollowJointTrajectory bridge for MyArm M750.",
    license="LicenseRef-Proprietary",
    entry_points={
        "console_scripts": [
            "driver_node = myarm_m750_driver.driver_node:main",
        ],
    },
)
