# Development plans — MyArm M750 SDK

## Cách sử dụng

Mỗi hạng mục phải có scope nhỏ, test/gate, dữ liệu debug và tiêu chí dừng. Không
đưa extension point vào critical path chỉ vì thư mục đã tồn tại.

Trạng thái:

- `[x]` hoàn tất và đã kiểm tra trong môi trường ghi rõ;
- `[~]` đang thực hiện;
- `[ ]` chưa bắt đầu;
- `[!]` bị chặn hoặc cần xác nhận phần cứng.

## Release 0.1.1 — repository và hybrid packaging

- [x] Đổi physical layout của `pycore/src` thành `api`, `application`, `domain`,
  `ports`, `adapters`, `runtime`, `diagnostics`.
- [x] Giữ distribution/import namespace là `myarm_m750_core` qua setuptools.
- [x] Đổi physical layout ROS 2 thành `description`, `driver`, `bringup`,
  `visualization`, `camera`, `moveit_config`, `gazebo`, `msgs`.
- [x] Thêm requirements profiles theo use case.
- [x] Thêm camera capture port, mock/OpenCV adapter và standalone camera session.
- [x] Thêm layout/camera unit tests.
- [x] Thêm `AGENTS.md`, `agent.md`, `plans.md` và cập nhật docs/changelog.
- [ ] ROS 2 graph test trên Foxy Jetson/Host.
- [ ] OpenCV camera test với thiết bị Logitech thật.
- [ ] Robot thật low-amplitude smoke test.

**Gate 0.1.1:** pure-Python tests và release verification đạt; không tuyên bố
ROS 2/robot/camera thật đã nghiệm thu khi chưa chạy.

## Release 0.1.2 — camera standalone ổn định

- [ ] Device discovery report theo `/dev/v4l/by-id`.
- [ ] FPS, dropped-frame, timeout và reconnect diagnostics.
- [ ] Calibration loader và camera intrinsic contract.
- [ ] Chạy độc lập một camera và hai camera.
- [ ] CLI ghi frame metadata/log JSONL, không bắt buộc GUI.
- [ ] Fault injection: mất camera, path sai, frame timeout, reconnect.

**Gate:** camera pipeline chạy không có ROS 2; lỗi camera không ảnh hưởng robot session.

## Release 0.1.3 — ROS 2 camera bridge và Host visualization

- [ ] `sensor_msgs/msg/Image` và `CameraInfo` bridge.
- [ ] TF camera frames theo hardware name.
- [ ] launch argument bật/tắt từng camera.
- [ ] image transport/QoS profile cho WLAN.
- [ ] RViz2 camera host config và diagnostics.

**Gate:** Host xem được một/hai camera qua DDS trong khi Jetson vẫn tự điều khiển robot.

## Release 0.2.0 — MoveIt 2 planning integration

- [ ] SRDF/planning group.
- [ ] Kinematics và joint-limit config.
- [ ] Controller/action mapping.
- [ ] Plan/validate trên Jetson; RViz2 chỉ visualize.
- [ ] Mock execution trước robot thật.
- [ ] FK/IK/trajectory consistency benchmark.

**Gate:** plan không bypass MotionGuard hoặc hardware adapter boundary.

## Extensions chưa cam kết version

- Gazebo Classic adapter và collision smoke test.
- MuJoCo standalone adapter và ROS bridge tùy chọn.
- C++/`ros2_control` chỉ khi benchmark chứng minh Python là bottleneck.
- Torque/dynamics/gravity compensation chỉ sau khi có inertial parameters đáng tin cậy.

## Template cho một task mới

```text
Task:
Owner/issue:
Problem:
In scope:
Out of scope:
Files/packages owned:
Safety risks:
Unit tests:
Integration test:
Logs/metrics:
Acceptance gate:
Rollback plan:
```
