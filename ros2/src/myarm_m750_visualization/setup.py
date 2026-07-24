from glob import glob
from setuptools import find_packages, setup

package_name = "myarm_m750_visualization"
setup(
    name=package_name,
    version="0.1.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nguyen Hoang Dang Khoa",
    maintainer_email="khoa@example.invalid",
    description="Host-PC RViz2 configuration and debug markers.",
    license="LicenseRef-Proprietary",
    entry_points={"console_scripts": ["marker_node = myarm_m750_visualization.marker_node:main"]},
)
