# MyArm M750 SDK — v0.1.1

Lớp controller giữa application/ROS 2 và phần cứng MyArm M750, phát triển theo
nguyên tắc: **đơn giản, từng bước, test trước robot thật, log được và mở rộng qua
ports/adapters**.

> Mặc định an toàn: `pycore/config/default.yaml` sử dụng mock adapter. Không đổi
> sang cấu hình robot thật trước khi xác nhận joint mapping, limits, serial device,
> nút dừng và vùng làm việc.

## Mô hình hybrid

Quan hệ phụ thuộc là một chiều:

```text
ROS 2 deployment
    ├── ROS 2 nodes, actions, topics, TF, RViz2
    └── imports/calls Python Core

Python Core standalone
    ├── public robot API
    ├── FK/IK/Jacobian, safety, trajectory
    ├── mock/replay/vendor adapters
    └── standalone camera pipeline
```

Nói cách khác:

- ROS 2 có thể chạy cùng và phụ thuộc vào `pycore`.
- `pycore` không phụ thuộc ROS 2 và vẫn chạy được ở mức control/kinematics/camera cơ bản.
- Camera capture nằm trong core; package ROS 2 camera chỉ bridge sang message, TF và diagnostics.

## Cấu trúc v0.1.1

```text
myarm_m750_sdk_v0.1.1/
├── AGENTS.md
├── agent.md
├── plans.md
├── requirements/
├── pycore/
│   ├── config/
│   ├── src/
│   │   ├── api/
│   │   ├── application/
│   │   ├── domain/
│   │   ├── ports/
│   │   ├── adapters/
│   │   ├── runtime/
│   │   └── diagnostics/
│   ├── tests/
│   └── pyproject.toml
├── ros2/
│   └── src/
│       ├── myarm_m750_description/
│       ├── myarm_m750_driver/
│       ├── myarm_m750_bringup/
│       ├── myarm_m750_visualization/
│       ├── myarm_m750_camera/
│       ├── myarm_m750_moveit_config/
│       ├── myarm_m750_gazebo/
│       └── myarm_m750_msgs/
├── examples/
├── benchmarks/
├── tools/
└── docs/
```

`pycore/src` là physical package root ngắn gọn. Khi cài bằng pip, setuptools ánh
xạ các thư mục này vào import namespace ổn định `myarm_m750_core.*`.

## Cài môi trường theo use case

### Core + test

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements/dev.txt
python3 -m pip install -e pycore
./tools/test_all.sh
```

### Camera standalone

```bash
python3 -m pip install -r requirements/camera.txt
python3 -m pip install -e pycore
python3 examples/camera_standalone_demo.py
```

Demo mặc định dùng mock camera, nên không cần ROS 2 hoặc webcam. Trên Jetson,
ưu tiên `requirements/camera-jetson.txt` và venv `--system-site-packages` để giữ
OpenCV của JetPack. Khi dùng OpenCV backend, sửa hardware serial và `/dev/v4l/by-id` trong
`pycore/config/camera/cameras.yaml`.

### Robot thật

```bash
python3 -m pip install -r requirements/serial.txt
python3 -m pip install -e pycore
```

Sau đó chỉ dùng `pycore/config/default_real.yaml` sau khi mock/URDF/mapping gate đã đạt.

### ROS 2

ROS 2, `rclpy`, message packages và tooling phải được cài bằng distribution/apt/
`rosdep`; không cài một bản `rclpy` tách rời bằng pip.


Trên Jetson chạy các lệnh:

```bash
python3 -m pip install -e pycore
cd ros2
source /opt/ros/foxy/setup.bash
rosdep install \
    --from-paths \
        src/myarm_m750_msgs \
        src/myarm_m750_description \
        src/myarm_m750_driver \
        src/myarm_m750_camera \
        src/myarm_m750_bringup \
    --ignore-src \
    --rosdistro foxy \
    -r \
    -y
colcon build \
    --symlink-install \
    --packages-select \
        myarm_m750_msgs \
        myarm_m750_description \
        myarm_m750_driver \
        myarm_m750_camera \
        myarm_m750_bringup
source install/setup.bash
```

Trên máy host chạy các lệnh. Phải dùng đúng `/usr/bin/python3` mà ROS Foxy
dùng để chạy node; `pip list` trong venv khác không làm ROS import được core.

```bash
/usr/bin/python3 -m pip install --user -e pycore
/usr/bin/python3 -c "import myarm_m750_core; print(myarm_m750_core.__file__)"
cd ros2
source /opt/ros/foxy/setup.bash 
rosdep install \
    --from-paths \
        src/myarm_m750_msgs \
        src/myarm_m750_description \
        src/myarm_m750_visualization \
        src/myarm_m750_gazebo \
    --ignore-src \
    --rosdistro foxy \
    -r \
    -y
colcon build \
    --symlink-install \
    --packages-select \
        myarm_m750_msgs \
        myarm_m750_description \
        myarm_m750_visualization \
        myarm_m750_gazebo 
source install/setup.bash
```

Mock bringup:

```bash
ros2 launch myarm_m750_bringup robot.launch.py \
  core_config_file:=$(realpath ../pycore/config/default.yaml)
```

Host RViz2:

```bash
ros2 launch myarm_m750_visualization rviz_host.launch.py
```

## Public Python API

Robot:

```python
from myarm_m750_core import RobotSession

with RobotSession.from_config("pycore/config/default.yaml") as robot:
    result = robot.move_joints(
        target=[0.20, -0.20, 0.15, 0.10, -0.10, 0.15],
        duration_s=3.0,
    )
    print(result.status.value, result.command_id)
```

Camera không ROS 2:

```python
from myarm_m750_core import CameraSession
from myarm_m750_core.adapters.camera import MockCameraAdapter

with CameraSession.from_config(
    "pycore/config/camera/cameras.yaml",
    "logitech_c922_01",
    capture=MockCameraAdapter(),
) as camera:
    frame = camera.read_one()
    print(frame.camera_name, frame.sequence, frame.image.shape)
```

## Quy ước không thay đổi

- `q_ros` là canonical coordinate cho URDF, TF, FK/IK, planner và simulation.
- Mapping firmware chỉ tại hardware adapter:

```text
q2_real = q2_ros + 10 deg
q3_real = q3_ros - 10 deg
```

- Đơn vị core dùng SI.
- `get_coords()` firmware không phải nguồn chân lý của pose.
- Mọi hardware command đi qua validation và explicit runtime state.
- Camera role không nằm trong hardware name; device ID là field riêng.

## Debug và test

- Unit test: `pycore/tests`.
- Release checks: `tools/verify_release.py`.
- JSONL log: command id, protocol/timeout context.
- Mock/replay adapter cho regression và fault injection.
- RViz2 target/trajectory markers.
- `AGENTS.md`: rule bắt buộc cho agent/contributor.
- `plans.md`: task, version plan và acceptance gate.

## Giới hạn đã biết

- ROS 2 graph, webcam thật và robot thật chưa được kiểm tra trong môi trường tạo ZIP.
- MoveIt 2, Gazebo và camera ROS bridge hiện là package/contract extension point.
- Velocity/effort/inertial values của URDF chưa phải dữ liệu nhà sản xuất đã chứng nhận.
- `firmware_speed` là vendor scalar, không phải rad/s.
- C++/`ros2_control` chưa được ưu tiên khi chưa có benchmark chứng minh bottleneck.
