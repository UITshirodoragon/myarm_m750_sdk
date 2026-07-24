# myarm_m750_camera

ROS 2 camera bridge extension point. Hardware discovery, stream configuration,
and standalone capture belong to Python Core under `pycore/config/camera` and
`pycore/src/adapters/camera`.

The ROS 2 package is responsible only for ROS interfaces such as
`sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo`, diagnostics, TF, and
launch composition. A camera failure must not transition the robot driver to
FAULT.
