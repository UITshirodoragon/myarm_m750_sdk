# Migration guide — v0.1.x sang v0.2.0

v0.2.0 là breaking release. Không có compatibility shim cho Python API, YAML,
ROS package hoặc asset cũ. Migration phải hoàn tất trên mock trước khi thử
profile thật.

## Python API

| v0.1.x | v0.2.0 |
| --- | --- |
| `RobotSession.from_config(path)` | `RobotSessionBuilder.from_file(path).build()` |
| composition nằm trong session | inject adapter/kinematics/scheduler tại builder |
| `get_state()` hoặc hardware getter mơ hồ | `read_joint_state()` / `read_hardware_status()` |
| primitive duration/options | immutable `MotionProfile` |
| `pause()` / `resume()` | bị xóa; dùng bounded cancel/stop sau khi capability được xác minh |
| dict/raw vendor object | typed value object và `CommandResult` |
| camera session tự compose | `CameraSessionBuilder.from_file(path).build()` |

Mọi lệnh chuyển động đi qua complete-trajectory admission. Code ứng dụng không
được gọi vendor object hoặc serial trực tiếp.

Ví dụ tối thiểu:

```python
from myarm_m750_core import MotionProfile, RobotSessionBuilder

builder = RobotSessionBuilder.from_file("/etc/myarm-m750/sdk.yaml")
with builder.build() as robot:
    result = robot.move_joints(
        [0.1, -0.1, 0.1, 0.0, 0.0, 0.0],
        MotionProfile(duration_s=3.0),
    )
```

## YAML

Mọi SDK root profile và file robot/safety/logging/camera do core sở hữu phải có
`config_version: 1`; unknown/missing field bị reject. Không đổi tên file cũ rồi
tiếp tục dùng default ngầm. ROS launch parameters, MoveIt config, model
manifest, calibration và network contract có schema/owner riêng; không thêm
`config_version` vào các file đó nếu loader của chúng không định nghĩa field này.

Profile SDK root tách reference tới ba owner:

```yaml
config_version: 1
sdk:
  config_files:
    robot: robot.yaml
    safety: safety.yaml
    logging: logging.yaml
  adapter:
    type: mock
    mock:
      initial_position_rad: [0, 0, 0, 0, 0, 0]
```

Profile robot thật phải khai báo:

- `/dev/serial/by-id/...`, baudrate và absolute-operation deadline;
- bounded retry/delay;
- expected model và firmware version/speed;
- SHA-256 mapping fingerprint;
- tri-state capability (`supported`, `unsupported`, `unverified`);
- `capabilities.verification_reference` truy vết report/fixture đã xác nhận
  capability; nếu có capability `supported` thì reference rỗng/placeholder bị
  reject;
- joint/model resource và kinematic-contract fingerprint.

`default_real.example.yaml` cố ý không chạy được. Sao chép ra deployment-owned
path và chỉ điền identity/limit đã được chứng nhận. Không commit serial hoặc
calibration của thiết bị cụ thể vào example.

Mapping fingerprint chỉ chứng minh contract phần mềm có thứ tự giữa YAML và
`JointMapper`; nó không tự chứng nhận dấu/offset vật lý trên từng robot.
Direction, zero offset và stop semantics vẫn phải giữ gate HIL `[!]` cho tới
khi có report trên thiết bị thật.

Safety config v0.2.0 còn bắt buộc budget `max_trajectory_points` và
`max_workspace_resample_samples`. Input vượt budget bị từ chối trước khi chạy
FK hoặc ghi waypoint; không tăng các giá trị này để né lỗi khi chưa đo tài
nguyên trên Jetson.

Camera dùng `cameras_mock.yaml` hoặc deployment copy của
`cameras_real.example.yaml`. Camera thật bắt buộc hardware serial,
`/dev/v4l/by-id`, calibration tương ứng resolution và extrinsic đã đo; không
fallback ngầm sang `/dev/video*`.

## ROS 2

Các package còn active:

- `myarm_m750_description`
- `myarm_m750_driver`
- `myarm_m750_bringup`
- `myarm_m750_visualization`
- `myarm_m750_moveit_config`
- `myarm_m750_camera`

`myarm_m750_msgs`, Gazebo scaffold, MuJoCo và Docker scaffold đã bị xóa.
Driver dùng standard `control_msgs/action/FollowJointTrajectory`,
`diagnostic_msgs`, `sensor_msgs`, `std_srvs` và explicit QoS. Foxy lifecycle
được biểu diễn bằng các Trigger service `configure`, `activate`, `deactivate`,
`cleanup`, `recover`.

Hai cờ sau mặc định `false` và phải khớp adapter:

```text
use_real_hardware
enable_command_interfaces
```

Remote RViz2 v0.2.0 khóa `rmw_fastrtps_cpp`. Host chỉ cài description và
visualization, không chạy core/driver/action server. Xem
`docs/deployment/remote-rviz2-wlan.md`.

## Model, mesh và kinematics

Không dùng standalone URDF cũ hoặc đường dẫn vào source ROS từ installed core.
Nguồn editable duy nhất là:

```text
myarm_m750_description/urdf/myarm_m750.urdf.xacro
```

Artifact `full`, `lightweight`, `kinematic` được generate và hash trong model
manifest. Core mang snapshot kinematic riêng. `tool0` giữ pitch `-π/2`.

PoE là provider mặc định. Chọn Pinocchio rõ ràng bằng
`builder.with_pinocchio()` trên Foxy/ARM64 có Pinocchio 2.6.17.
`pytransform3d==3.16.0` chỉ thuộc extra `geometry-tools` cho inspection; không
dùng trong control loop. Không thêm spatialmath-python trong release này.
Target IK phải mang đúng frame canonical `base_link→tool0`; target frame khác
bị reject thay vì được ngầm diễn giải hoặc đổi frame.

Full DAE hiện vượt performance budget; dùng `lightweight` cho MoveIt/mock/Host
cho tới khi decimation, license và visual QA được duyệt. Dynamics, gravity và
torque vẫn tắt.

## Dependency và validation

Pip dependency chỉ quản lý qua `pycore/pyproject.toml` và
`requirements/constraints-py38.txt`. ROS/system dependency dùng apt/rosdep.
Không cài `rclpy` hoặc Pinocchio bằng pip.

Sau migration:

```bash
./tools/bootstrap_core.sh
./tools/test_core.sh
./tools/bootstrap_ros.sh
./tools/test_ros.sh
.venv-core/bin/python tools/verify_release.py
```

`tools/verify_release.py --release-ready` còn kiểm tra artifact blocker như
mesh budget. Gate robot, camera thật và WLAN hai máy chỉ được đóng bằng report
trên đúng deployment, không bằng mock.
