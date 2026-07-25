from setuptools import find_packages, setup

package_name = "myarm_m750_camera"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", ["config/camera_bridge.yaml"]),
        ("share/" + package_name + "/launch", ["launch/camera_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nguyen Hoang Dang Khoa",
    maintainer_email="khoa@example.invalid",
    description="Thin ROS 2 Image/CameraInfo/TF/diagnostics bridge.",
    license="LicenseRef-Proprietary",
    entry_points={
        "console_scripts": [
            "camera_bridge = myarm_m750_camera.camera_bridge:main",
        ],
    },
)
