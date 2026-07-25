# ROS 2 workspace

ROS 2 is the deployment and integration layer. It is not the owner of robot
kinematics, safety, trajectory generation, hardware mapping, or standalone
camera capture.

Install Python Core with the exact interpreter used by ROS 2 before building.
On ROS Foxy this is normally `/usr/bin/python3`; do not rely on `pip list` from
an activated virtual environment.

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3 -m pip install --user -e ../pycore
/usr/bin/python3 -c "import myarm_m750_core; print(myarm_m750_core.__file__)"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The physical package folder and package name are identical:

```text
myarm_m750_description
myarm_m750_driver
myarm_m750_bringup
myarm_m750_visualization
myarm_m750_camera
myarm_m750_moveit_config
myarm_m750_gazebo
myarm_m750_msgs
```
