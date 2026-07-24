from glob import glob
from setuptools import find_packages, setup

package_name = "myarm_m750_driver"

setup(
    name=package_name,
    version="0.1.1",
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
    description="ROS 2 bridge for the MyArm M750 Python Core SDK.",
    license="LicenseRef-Proprietary",
    entry_points={
        "console_scripts": [
            "driver_node = myarm_m750_driver.driver_node:main",
        ],
    },
)
