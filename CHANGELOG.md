# Changelog

Tất cả thay đổi đáng chú ý của SDK được ghi tại đây. Repository dùng Semantic
Versioning.

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
- `agent.md` compatibility entry point và `plans.md` cho versioned task/gate planning.
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
