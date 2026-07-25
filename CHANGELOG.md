# Changelog

Tất cả thay đổi đáng chú ý của SDK được ghi tại đây. Repository dùng Semantic
Versioning.

## 0.2.0 — Unreleased

### Breaking

- Thay composition API bằng `RobotSessionBuilder`/`CameraSessionBuilder`; xóa
  `from_config()`, `get_state()`, `pause()` và `resume()`.
- Cấu hình chuyển sang strict `config_version: 1`; legacy/unknown/missing field
  bị từ chối trước khi mở hardware.
- Hardware boundary dùng value object và absolute monotonic deadline; capability
  có `SUPPORTED`, `UNSUPPORTED`, `UNVERIFIED`; capability thật được quảng cáo
  phải có probe và evidence reference truy vết được.
- Xóa custom message, Gazebo, MuJoCo và Docker scaffold không có consumer.

### Added

- Tách command admission, pure complete-trajectory validator, absolute-deadline
  executor, cancellation generation, watchdog, workload budget và metrics.
- ROS 2 Foxy driver có lifecycle-equivalent Trigger services, standard
  `FollowJointTrajectory`, explicit QoS và diagnostics ngay khi inactive.
- Fast DDS WLAN profiles, Jetson/Host launch tách biệt và network probe
  JSON/CSV; local gate lấy tối thiểu 100 state sample, remote command mặc định
  tắt.
- PoE reference, Pinocchio 2.6.17 provider, optional pytransform3d 3.16.0,
  shared DLS IK và golden 128-configuration comparison.
- Canonical Xacro sinh full/lightweight/kinematic variants, model manifest,
  packaged core resource và MoveIt Foxy plan/mock-execution package.
- Independent camera workers, depth-one latest queues, reconnect/metrics,
  calibration/extrinsic validation và ROS Image/CameraInfo/static-TF bridge.

### Verification scope

- Source, mock, vendor-fake và ROS local/headless gates thuộc release gate.
- Gate hiện tại: minimal core 245 pass/2 Pinocchio-system skip, coverage
  93.02%, Ruff/mypy
  zero; ROS Foxy build 6/6 package, 11 test-result records không lỗi và live
  mock driver/P4a/two-camera graph đạt.
- MoveIt plan-only, collision rejection và mock execution đạt qua automated
  live gate có driver-readiness, conservative time-scaling và clean shutdown;
  MoveIt-level cancel trên Foxy vẫn mở vì cancel chỉ được xử lý sau terminal.
- Robot/camera thật và WLAN hai máy vẫn cần bằng chứng `[!]`; dynamics/torque
  bị vô hiệu vì inertial/payload chưa có provenance.
- Detailed visual meshes vượt release performance budget; lightweight variant
  dùng primitive, còn decimation/license/visual QA giữ `[!]`.

## 0.1.1 — 2026-07-24

### Changed

- Chuyển physical Python Core layout thành `pycore/src/api`, `application`,
  `domain`, `ports`, `adapters`, `runtime`, `diagnostics`.
- Giữ import namespace tương thích `myarm_m750_core.*` bằng explicit setuptools mapping.
- Chuyển physical ROS 2 layout thành `description`, `driver`, `bringup`,
  `visualization`, `camera`, `moveit_config`, `gazebo`, `msgs`.
- Làm rõ deployment hybrid: ROS 2 được phép phụ thuộc Python Core; Python Core
  không phụ thuộc ROS 2.
- Chuyển config phần cứng camera thành ownership của `pycore/config/camera`;
  ROS 2 camera chỉ giữ bridge parameters.

### Added

- `AGENTS.md` làm coding-agent/contributor contract dựa trên clean-code,
  clean-comment, safety và ROS 2 boundary rules.
- Canonical contributor/planning documents được thêm; compatibility aliases của
  release đó đã bị xóa trong breaking release 0.2.0.
- `requirements/` profiles: base, dev, camera, serial, simulation, ROS 2 và all.
- ROS-independent camera domain models, capture port, mock adapter, optional OpenCV
  adapter, `CameraPipeline` và public `CameraSession`.
- Standalone camera demo không cần ROS 2.
- Unit tests cho camera standalone và physical repository layout.
- ROS 2 extension packages cho MoveIt 2, Gazebo và custom messages.

### Verification scope

- Pure-Python unit tests và source-only release checks được chạy trong môi trường tạo release.
- Chưa tuyên bố đạt cho ROS 2 graph, OpenCV webcam thật hoặc robot thật.

## 0.1.0 — 2026-07-24

### Added

- Public API `RobotSession`; Application không gọi raw firmware API.
- Canonical joint coordinates theo ROS, SI units trong Python Core.
- Adapter `mock`, `replay` và `vendor_serial` với mapping q2/q3 chỉ tại hardware boundary.
- URDF PoE v3.2 nguyên bản do dự án cung cấp và một URDF primitive độc lập cho RViz2.
- Software PoE FK, Jacobian, singularity score và numerical IK bằng NumPy.
- Joint/workspace/singularity/stale-state validation tập trung trong `MotionGuard`.
- Cubic point-to-point trajectory và executor mặc định 5 Hz.
- Explicit runtime state machine, stop/cancel coordination và single-command ownership.
- Structured console logging và rotating JSONL file logging.
- ROS 2 Foxy packages: description, driver, bringup và Host-PC RViz2 visualization.
- `FollowJointTrajectory`, `/joint_states`, `/diagnostics`, RobotModel, TF và debug markers.
- Camera package ở mức extension point.
- Unit tests, adapter contract tests, URDF contract tests và release static checks.
- B1/T1 mock waypoint và B2 FK/IK benchmark examples.

### Deliberately deferred

- Full MoveIt 2 configuration and execution integration.
- Complete camera acquisition node and reconnect implementation.
- Gazebo Classic and MuJoCo adapters.
- C++/`ros2_control`, torque control, dynamics, gravity compensation and force control.

### Known limitations

- Velocity/effort and inertial values in the supplied model are not manufacturer-certified.
- `firmware_speed` remains a vendor scalar, not a physical velocity in rad/s.
- Pause/resume does not suspend and continue a software trajectory in 0.1.0; cancel or stop instead.
- ROS graph tests require a real ROS 2 Foxy environment and are not executed by the pure-Python CI.
