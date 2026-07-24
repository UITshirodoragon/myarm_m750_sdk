# Implementation notes — v0.1.1

## Thiết kế được khóa

| Quyết định | Triển khai |
|---|---|
| ROS-independent core | physical `pycore/src/*`, installed namespace `myarm_m750_core.*`; không import `rclpy` |
| ROS 2 hybrid | ROS packages gọi public core API; core không gọi ROS 2 |
| SI units | domain/public API dùng rad, m, s, Hz |
| Canonical `q_ros` | joint order lấy từ `robot_m750.yaml` và URDF |
| Mapping tại adapter | `pycore/src/adapters/joint_mapping.py` |
| Software kinematics | `pycore/src/domain/kinematics/poe.py` |
| Safety tập trung | `pycore/src/domain/safety/motion_guard.py` |
| Joint trajectory 5 Hz | `runtime/trajectory.py` + `runtime/executor.py` |
| Explicit robot state | `runtime/state_machine.py` |
| Traceable result | `CommandResult` + UUID `command_id` |
| Camera standalone | `CameraSession` → `CameraPipeline` → `CameraCapturePort` |
| Camera optional backend | mock hoặc lazy-import OpenCV adapter |
| Jetson execution | ROS packages `driver` + `bringup` |
| Host visualization | ROS package `visualization` |
| Config ownership | `pycore/config` và config riêng của ROS bridge package |

## Physical layout và package namespace

Thư mục `pycore/src` không chứa wrapper `myarm_m750_core/`. `pyproject.toml`
ánh xạ explicit physical root thành package `myarm_m750_core`, nên public import
không đổi:

```python
from myarm_m750_core import RobotSession, CameraSession
```

Cách này giữ layout theo thiết kế nhưng vẫn cho phép pip/ROS 2 import một
namespace duy nhất. Test và script phải cài package editable trước; không thêm
trực tiếp `pycore/src` vào `PYTHONPATH`.

## Camera boundary

Hardware camera config nằm ở `pycore/config/camera/cameras.yaml`. ROS package
`ros2/src/myarm_m750_camera` chỉ giữ bridge parameters và interface dependencies. Camera
lifecycle/fault tách khỏi robot runtime state. Application cấp cao mới quyết định
một vision fault có cần dừng task hay không.

## URDF contract

`myarm_m750_poe_v3_2.urdf` được giữ nguyên. Python Core parse chuỗi
`base_link -> tool0` để suy ra screw axes, home transform và joint limits; không
hard-code một FK khác.

## Error boundary

- Config sai: `ConfigurationError` khi startup.
- Vendor không import/kết nối được: `HardwareConnectionError`.
- `-1`, `None`, malformed reply: `ProtocolError`.
- Safety: command `REJECTED`, có `error_code` và message.
- Runtime exception: command `FAILED`, robot driver chuyển `FAULT`.
- Camera read timeout/error: camera result/exception riêng, không fault robot core.
