# ROS 2 workspace

ROS 2 is the deployment and integration layer. It is not the owner of robot
kinematics, safety, trajectory generation, hardware mapping, or standalone
camera capture.

Install Python Core into the same Python environment before building:

```bash
python3 -m pip install -e ../pycore
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Physical package folders are short, while package names remain explicit in
`package.xml`:

```text
description     -> myarm_m750_description
driver          -> myarm_m750_driver
bringup         -> myarm_m750_bringup
visualization   -> myarm_m750_visualization
camera          -> myarm_m750_camera
moveit_config   -> myarm_m750_moveit_config
gazebo          -> myarm_m750_gazebo
msgs            -> myarm_m750_msgs
```
