# AGENTS.md — MyArm M750 SDK

Tài liệu này là contract làm việc cho coding agent và người đóng góp. Mọi thay
đổi phải ưu tiên theo thứ tự: **safety/correctness → traceability → readability
→ consistency → local convention → personal preference**.

## 1. Phạm vi kiến trúc

```text
Application / ROS 2 node / benchmark
                ↓
        public API + application
                ↓
       domain + runtime + safety
                ↓
                 ports
                ↓
              adapters
                ↓
      firmware / camera / simulator
```

Các quy tắc không được phá:

- `pycore` không import `rclpy` và phải chạy được khi ROS 2 chưa được cài.
- ROS 2 package chỉ bridge interface, compose lifecycle và gọi public API/core.
- Domain/kinematics không gọi serial, camera, filesystem hoặc ROS 2.
- Application không gọi raw vendor API.
- Mapping `q_ros ↔ q_real` chỉ tồn tại ở hardware adapter.
- Camera capture/core pipeline phải chạy standalone; ROS 2 camera chỉ là bridge.
- Camera fault không được tự động chuyển robot driver sang `FAULT`.
- Mặc định luôn dùng mock adapter; robot thật phải được bật bằng config rõ ràng.

## 2. Cấu trúc repository bắt buộc

```text
pycore/
├── config/
├── src/
│   ├── api/
│   ├── application/
│   ├── domain/
│   ├── ports/
│   ├── adapters/
│   ├── runtime/
│   └── diagnostics/
├── tests/
└── pyproject.toml

ros2/src/
├── myarm_m750_description/
├── myarm_m750_driver/
├── myarm_m750_bringup/
├── myarm_m750_visualization/
├── myarm_m750_camera/
├── myarm_m750_moveit_config/
├── myarm_m750_gazebo/
└── myarm_m750_msgs/
```

`pycore/src` được setuptools ánh xạ thành namespace cài đặt
`myarm_m750_core.*`. Không tạo lại thư mục vật lý `src/myarm_m750_core` trừ khi
có ADR mới và migration plan.

## 3. Quy tắc code

- Tên mang domain và đơn vị: `joint_position_rad`, `timeout_s`, `rate_hz`.
- Không dùng magic number cho giới hạn vật lý/timing/protocol.
- Hàm nhỏ, một mức trừu tượng; tên có chữ `and` thường là dấu hiệu cần tách.
- Tách computation khỏi side effect. `compute_*`, `solve_*`, `validate_*` không
  được gửi lệnh hoặc publish.
- Boundary condition được tập trung trong validator/guard/config owner.
- Tránh flag argument; tạo API riêng cho hành vi riêng.
- Ưu tiên immutable value object thay vì nhiều primitive rời rạc.
- Predicate safety dùng dạng dương: `is_safe`, `succeeded`, `is_ready`.
- Runtime state phải explicit; không thay state machine bằng cụm boolean ẩn.
- Exception/error phải có taxonomy và context; không trả `-1`/`None` mơ hồ.
- Public API có type hints và docstring nêu input, unit, return, exception và side effect.

## 4. Comment

Comment chỉ giữ thông tin code không tự nói được:

- intent/why;
- constraint phần cứng, timing hoặc frame;
- warning và hậu quả;
- firmware workaround có issue/điều kiện xóa;
- nguồn thuật toán/chuẩn;
- TODO có owner hoặc issue;
- lý do regression test tồn tại.

Không comment lặp code, không dùng comment để che tên xấu, không giữ code đã
comment-out. Git lưu lịch sử.

## 5. ROS 2

- Node chỉ compose parameters, publishers/subscribers/actions, lifecycle và core API.
- Callback trên command path phải bounded; không block serial/camera/network tùy ý.
- Parameter phải có unit/range trong tên hoặc mô tả.
- Topic/frame/QoS contract được ghi tại interface boundary.
- Ưu tiên standard message; custom interface chỉ đặt trong `ros2/src/myarm_m750_msgs` khi
  standard message không diễn đạt đủ contract.
- Build package sau khi thay dependency, launch hoặc interface; luôn source lại workspace.

## 6. Camera

- Tên instance dựa trên phần cứng: `logitech_c922_01`.
- `role`, `hardware_serial`, `device.by_id` là field riêng; không nhét vào tên.
- Ưu tiên `/dev/v4l/by-id`; fallback `/dev/video*` phải explicit.
- OpenCV là optional dependency; control-only deployment không được kéo camera stack.
- Unit test camera dùng mock backend, không phụ thuộc thiết bị thật.

## 7. Quy trình thay đổi

1. Đọc `PLANS.md`, docs thiết kế và file lân cận.
2. Viết/điều chỉnh test trước hoặc cùng lúc với implementation.
3. Thay đổi nhỏ, có một mục tiêu; không refactor ngoài scope.
4. Chạy `./tools/test_all.sh`.
5. Cập nhật `CHANGELOG.md`, docs và version nếu tạo release.
6. Chạy `python3 tools/verify_release.py`.
7. Không tuyên bố robot thật/ROS graph đã đạt nếu chưa chạy trên môi trường đó.

## 8. Definition of done

Một thay đổi chỉ hoàn tất khi:

- test mới và regression test đều đạt;
- dependency direction không bị phá;
- log/error có đủ context để debug;
- config có owner rõ và được validate fail-fast;
- README/changelog/plans phản ánh đúng trạng thái;
- generated cache/log/build artifacts không nằm trong release ZIP.
