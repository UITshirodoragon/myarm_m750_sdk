# Verification playbook — MyArm M750 SDK v0.2.0

Không dùng tài liệu này để tuyên bố gate hardware đã đạt. Kết quả chính thức và
trạng thái `[x]`/`[!]` được ghi trong `PLANS.md` cùng command, môi trường và
artifact tương ứng.

## Core source/mock gate

Target: ARM64, Ubuntu 20.04, Python 3.8.

```bash
./tools/bootstrap_core.sh
./tools/test_core.sh
```

Gate chạy trong `.venv-core`, tắt pytest plugin autoload và không ghi user site:

- Ruff và mypy;
- unit/integration test với coverage toàn core tối thiểu 85%;
- coverage statement tối thiểu 90% riêng cho bốn nhóm không chồng lấn:
  `runtime/config`, `domain/safety`, `runtime` (không gồm config) và `adapters`;
- source release verifier;
- byte-code compilation;
- wheel build, isolated target install, import smoke và composition smoke hoàn
  toàn từ resource đã đóng gói (strict config → builder → mock session → state).

`tools/verify_release.py` kiểm tra canonical `AGENTS.md`/`PLANS.md`, physical
layout, inventory owner/consumer, version, YAML/XML, dependency direction, safe
default, dependency policy và deterministic model artifacts. Source gate dùng:

```bash
.venv-core/bin/python tools/verify_release.py
```

Trước khi đóng gói/tag release, bắt buộc chạy thêm:

```bash
.venv-core/bin/python tools/verify_release.py --release-ready
```

Chế độ `--release-ready` còn enforce budget asset; source verifier xanh không
được diễn giải thành artifact release đã sẵn sàng.

## ROS 2 Foxy local/headless gate

ROS dùng môi trường riêng vì `launch_testing` của Foxy không tương thích pytest
8 trong core environment:

```bash
./tools/bootstrap_ros.sh
source /opt/ros/foxy/setup.bash
rosdep install --from-paths ros2/src --ignore-src --rosdistro foxy -r -y
./tools/test_ros.sh
```

`.venv-ros` dùng `/usr/bin/python3`, `--system-site-packages` và pytest 6.2.5.
Gate build/test đúng sáu package active; custom-message và simulator scaffold
không thuộc release. Gate cũng phải chạy test Pinocchio thật bằng package apt
Foxy 2.6.17; test vendor-fake không thay bằng chứng native này.

Sau colcon test, `tools/test_ros.sh` gọi `tools/ros_runtime_gate.py` trên
install-space vừa build. Harness phải xác nhận lifecycle diagnostics khi chưa
active, FollowJointTrajectory feedback/direct cancel trong lúc state publisher
vẫn chạy, exact `/robot_description`, đủ canonical `/tf`/`/tf_static`, P4a
network JSON/CSV budget sau tối thiểu 100 joint-state sample và hai camera
Image/CameraInfo/static-TF/diagnostics với shutdown bounded.
`tools/moveit_runtime_gate.py` sau đó chạy domain riêng cho plan-only và
mock-execution, yêu cầu collision rejection, trajectory time tăng nghiêm ngặt,
execution success và shutdown child sạch. `mock-cancel` được ghi rõ là blocker,
không được bỏ qua bằng cách trộn nó vào passing report. Fast DDS cần quyền tạo
graph loopback và đọc interface (`getifaddrs`); môi trường CI sandbox chặn
network syscall phải cung cấp runner ROS phù hợp, không được bỏ qua live gate.

## Gate cần môi trường ngoài source tree

Các mục sau luôn giữ `[!]` cho tới khi có log/report từ đúng hệ thống:

- MyArm M750 thật: identity, firmware, stop/cancel, low-amplitude motion và HIL;
- camera thật: serial/by-id, calibration, unplug/replug và JetPack/OpenCV;
- Jetson ↔ Host qua WLAN: discovery, latency/age, reconnect, clock offset và
  bandwidth;
- MoveIt execution trên robot thật và collision geometry review;
- MoveIt-level cancel trên Foxy (direct driver-action cancel không thay cho
  cancel qua `move_group`);
- license/provenance của mesh, limits, inertial và payload.
