# Implementation notes — v0.2.0

## Contract đã triển khai

| Boundary | v0.2 implementation |
|---|---|
| Composition | `RobotSessionBuilder` và `CameraSessionBuilder`; build không mở device |
| Robot API | lifecycle, `read_*`, FK/Jacobian, admitted execution, cancel/stop/recover |
| Config | strict `config_version: 1`, reject missing/unknown/legacy trước I/O |
| Hardware | typed profiles, `CommandContext`, absolute monotonic deadline, tri-state capabilities + evidence reference |
| Safety | bounded `TrajectoryValidator` thuần → `CommandAdmission` → `TrajectoryExecutor` |
| Runtime | event/reason/command state history, cancellation generation, state-read watchdog và metrics |
| Kinematics | PoE reference; Pinocchio 2.6.17 optional provider; shared DLS IK |
| Model | one Xacro → full/lightweight/kinematic + normalized contract manifest |
| Camera | independent worker/camera, latest queue depth 1, reconnect, calibration/extrinsics |
| ROS camera | direct NumPy `Image`, `CameraInfo`, static TF, diagnostics; không cần `cv_bridge` |
| WLAN | Jetson execution vs Host observe-only, Fast DDS profiles, JSON/CSV probe |

Physical `pycore/src/*` vẫn được setuptools ánh xạ vào
`myarm_m750_core.*`. Wheel chứa kinematic URDF và manifest riêng, nên core không
resolve ngược sang source/share của ROS.

## Model và geometry

PoE là provider mặc định/reference. Pinocchio chỉ fallback về PoE khi import/ABI
không khả dụng; model/frame/joint mismatch là startup error. Jacobian public là
end-link origin trong base frame, thứ tự `[angular, linear]`. Dynamics, gravity
và torque luôn disabled vì inertial/payload chưa có provenance.

IK chỉ nhận target đúng frame contract `base_link→tool0`. Safety config giới
hạn mặc định 1.000 trajectory point và 10.000 workspace-resampling FK sample;
vượt budget bị reject trước vòng FK/I/O.

`pytransform3d==3.16.0` chỉ thuộc extra `geometry-tools` cho model/frame/mesh
inspection, không nằm trong control loop. Release không thêm spatialmath-python,
Open3D hoặc MeshCat.

## Camera và timestamp

Mock config là `pycore/config/camera/cameras_mock.yaml`. File
`cameras_real.example.yaml` cố ý không có camera chạy được; deployment phải đưa
hardware serial/by-id, calibration và extrinsics đã chứng nhận vào file riêng.
Frame giữ acquisition monotonic time cho freshness và observation wall time cho
ROS timestamp.

## Bằng chứng và gate còn mở

- Minimal core gate hiện có 245 pass, 2 Pinocchio-system skip, coverage 93.02%,
  Ruff/mypy zero và wheel composition smoke pass. Native Pinocchio được chạy
  riêng trong ROS gate (7 pass). ROS Foxy local/headless build 6/6 package với
  11 test-result records không lỗi; driver 25, camera 6 và visualization 14
  test case đạt.
- Live install-space gate đạt driver lifecycle/action feedback/direct cancel,
  exact canonical description, 8 dynamic + 2 static TF edge, P4a JSON/CSV
  budget trên cửa sổ tối thiểu 100 joint-state sample và bridge hai camera.
  Automated MoveIt gate đạt plan-only, collision reject và mock execution qua
  driver; readiness/time-scaling và clean child shutdown đều được kiểm tra.
  MoveIt-level cancel trên Foxy chỉ được xử lý sau terminal
  (`GOAL_TERMINATED`) nên Phase 6 vẫn `[~]`.
- Robot, webcam/JetPack, unplug/replug và WLAN hai máy giữ `[!]` đến khi có
  phần cứng và report đo.
- OpenCV/V4L `capture.read()` có hành vi block phụ thuộc backend/driver; bounded
  shutdown API đã có nhưng vẫn phải qua unplug/stall gate trên JetPack trước khi
  tuyên bố camera thật đạt.
- Detailed visual assets hiện vượt 40 MiB/300k triangles; lightweight primitive
  variant dùng được, còn decimation, mesh license và visual QA giữ `[!]`.
