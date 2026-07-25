# ROS 2 workspace

ROS 2 is the deployment and integration layer. It is not the owner of robot
kinematics, safety, trajectory generation, hardware mapping, or standalone
camera capture.

On Jetson, install Python Core only into the repository-owned ROS environment.
It uses the exact `/usr/bin/python3` interpreter and system packages from Foxy.
Never write the user site to make ROS imports work.

```bash
./tools/bootstrap_ros.sh
source /opt/ros/foxy/setup.bash
rosdep install --from-paths ros2/src --ignore-src --rosdistro foxy -r -y
./tools/test_ros.sh
```

The active physical package folder and package name are identical:

```text
myarm_m750_description
myarm_m750_driver
myarm_m750_bringup
myarm_m750_visualization
myarm_m750_camera
myarm_m750_moveit_config
```

The Host PC only needs `myarm_m750_description` and
`myarm_m750_visualization`. It must not install or launch Python Core or a
second hardware driver for observe-only RViz deployment.
