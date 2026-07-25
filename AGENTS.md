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
- Session/driver thật chỉ được chuyển sang trạng thái nhận command sau khi
  `probe_hardware()` đã xác minh identity, firmware response, joint count,
  mapping fingerprint và capability cần dùng. Khai báo YAML không phải bằng
  chứng capability.
- Mapping fingerprint chỉ chứng minh software mapping có thứ tự giữa config và
  adapter; dấu/offset vật lý vẫn cần calibration/HIL report, không được suy ra
  từ hash.
- Mọi profile thật có command interface phải có stop capability ở trạng thái
  `SUPPORTED` kèm `verification_reference` truy vết được và probe runtime xác
  nhận đúng firmware/method. Không có bằng chứng stop thì chỉ được
  observe/read-only; không được suy diễn capability từ việc method tồn tại.
- Jetson/robot computer sở hữu execution và safety; Host PC qua WLAN chỉ
  visualization/observability theo mặc định. Mất Host hoặc mất WLAN không được
  làm dừng hay fault control loop cục bộ.
- Thư viện robotics bên thứ ba phải nằm sau port/adapter/provider boundary;
  không để Pinocchio, MoveIt, OpenCV hoặc simulator trở thành dependency bắt
  buộc của control-only core.

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
│   ├── diagnostics/
│   └── resources/
├── tests/
└── pyproject.toml

ros2/src/
├── myarm_m750_description/
├── myarm_m750_driver/
├── myarm_m750_bringup/
├── myarm_m750_visualization/
├── myarm_m750_camera/
└── myarm_m750_moveit_config/
```

`pycore/src` được setuptools ánh xạ thành namespace cài đặt
`myarm_m750_core.*`. Không tạo lại thư mục vật lý `src/myarm_m750_core` trừ khi
có ADR mới và migration plan.

## 3. Quy tắc code

- Code production phải parse và chạy trên Python 3.8. Không dùng cú pháp chỉ có
  từ Python 3.9/3.10 trở lên, kể cả khi type checker chấp nhận.
- Tên mang domain và đơn vị: `joint_position_rad`, `timeout_s`, `rate_hz`.
- Với pose/transform, tên hoặc type phải nói rõ source frame, target frame và
  quaternion order; không truyền một `pose`, `transform` hoặc `quat` mơ hồ qua
  boundary.
- Không dùng magic number cho giới hạn vật lý/timing/protocol.
- Hàm nhỏ, một mức trừu tượng; tên có chữ `and` thường là dấu hiệu cần tách.
- Tách computation khỏi side effect. `compute_*`, `solve_*`, `validate_*` không
  được gửi lệnh, đọc hardware hoặc publish. Dùng `read_*`/`poll_*` cho query và
  `send_*`/`write_*`/`execute_*`/`publish_*` cho mutation hay I/O.
- Boundary condition được tập trung trong validator/guard/config owner.
- Tránh flag argument; tạo API riêng cho hành vi riêng.
- Ưu tiên immutable value object thay vì nhiều primitive rời rạc.
- Predicate safety dùng dạng dương: `is_safe`, `succeeded`, `is_ready`.
- Structural safety không có công tắc tắt: finite value, joint set/order,
  monotonic time, dimension và limit provenance luôn phải được kiểm tra. Nếu
  derivative được caller cung cấp, validator vẫn phải đối chiếu với derivative
  suy ra từ position/time; dữ liệu tự khai báo không được che chuyển động quá
  giới hạn.
- Số trajectory point và tổng workspace-resampling sample phải có budget dương,
  hữu hạn trong config. Reject vượt budget trước vòng FK/I/O đầu tiên để input
  ngoài không thể tạo công việc CPU/bộ nhớ không giới hạn.
- Runtime state phải explicit; không thay state machine bằng cụm boolean ẩn.
- Stop/cancel chỉ được ghi nhận thành công khi adapter xác nhận thành công. Một
  lần stop bị reject/timeout không được cache thành success và phải dẫn tới
  terminal state/fault có lý do rõ.
- Exception/error phải có taxonomy và context gồm operation, command ID,
  adapter/device và deadline khi áp dụng; không trả `-1`/`None` mơ hồ.
- Timeout, retry và vòng lặp phải bounded. Deadline/freshness runtime dùng
  monotonic clock; wall clock chỉ dùng cho thời gian quan sát/log.
- Dữ liệu từ filesystem, YAML, serial, camera, ROS và native library phải được
  validate tại boundary trước khi thành domain object.
- Không truy cập thuộc tính private của dependency, trừ workaround có version
  guard, issue, hậu quả và điều kiện xóa.
- Public API có type hints và docstring nêu input, unit/frame, return, exception
  và side effect. Internal API có annotation đủ để mypy kiểm tra call path.
- Không tối ưu hoặc thêm abstraction nếu chưa có invariant, consumer hoặc
  benchmark chứng minh nhu cầu; ưu tiên luồng dễ trace từ command tới feedback.

## 4. Comment

Comment chỉ giữ thông tin code không tự nói được:

- intent/why;
- constraint phần cứng, timing, register/bit/errata hoặc frame;
- warning nêu rõ hậu quả và context không được gọi;
- firmware/vendor workaround có issue, version/điều kiện kích hoạt và điều kiện xóa;
- nguồn thuật toán/chuẩn, version và phần đã chuyển thể;
- TODO có owner hoặc issue;
- lý do regression test tồn tại.

Không comment lặp code, không dùng comment để che tên xấu, không giữ code đã
comment-out. Comment sai hoặc hết hạn phải được sửa/xóa cùng code. Git lưu lịch
sử. Tài liệu
`docs/development/clean-code-clean-comment-robotics.qmd` giữ checklist và ví dụ
review đầy đủ; các quy tắc bắt buộc được cô đọng tại đây.

## 5. ROS 2

- Node chỉ compose parameters, publishers/subscribers/actions, lifecycle và core API.
- Callback trên command path phải bounded; không block serial/camera/network tùy ý.
- Parameter phải có unit/range trong tên hoặc mô tả.
- Topic/frame/QoS contract được ghi tại interface boundary.
- Ưu tiên standard message. Custom interface chỉ được thêm lại qua ADR chứng
  minh standard message không diễn đạt đủ contract và có owner/consumer/test.
- Build package sau khi thay dependency, launch hoặc interface; luôn source lại workspace.
- Cancellation phải được nghiệm thu tại đúng boundary được quảng bá. Driver
  action cancel đạt không được dùng thay bằng chứng MoveIt
  `/execute_trajectory`/`/move_action` cancel.

### 5.1 Remote visualization Jetson ↔ Host qua WLAN

Deployment mặc định:

```text
MyArm + Jetson                         Host PC
driver/core/safety/TF                  RViz2/diagnostics
        └──────── ROS 2 DDS / WLAN ─────────┘
```

- Jetson publish tối thiểu `/joint_states`, `/tf`, `/tf_static`,
  `robot_description` và diagnostics; Host chạy
  `myarm_m750_visualization` + RViz2, không chạy hardware driver thứ hai.
- Jetson và Host phải có ROS distro, message definitions, `ROS_DOMAIN_ID` và
  RMW implementation tương thích. Mọi launch/runbook phải ghi rõ các giá trị này.
- Release v0.2.0 khóa `rmw_fastrtps_cpp`. Discovery phải explicit: multicast
  chỉ dùng khi WLAN/AP hỗ trợ ổn định; nếu không, dùng generated Fast DDS
  peer/unicast hoặc Discovery Server profile. Không giả định hai máy tự
  discover chỉ vì ping được nhau.
- QoS được chọn theo loại dữ liệu: state/TF/camera ưu tiên freshness và bounded
  queue; command/action cần reliability và timeout. Không dùng một QoS profile
  cho mọi topic.
- Đồng bộ thời gian bằng chrony/NTP và theo dõi clock offset; TF timestamp lỗi
  không được che bằng cách tăng queue vô hạn.
- Host loss, DDS discovery loss hoặc packet loss không thay đổi safety state của
  Jetson. Diagnostics phải phân biệt network degradation với hardware fault.
- Remote command từ Host mặc định tắt. Nếu bật, phải có config explicit,
  namespace/domain isolation, authentication/VPN hoặc SROS2 phù hợp, command
  arbitration, rate limit và local E-stop vẫn có quyền cao nhất.
- Không stream mesh/model lặp lại ở runtime; description/mesh được cài trên cả
  hai máy. Camera phải có image transport, resolution/FPS và bandwidth budget.
- Acceptance bắt buộc đo discovery time, end-to-end state latency, packet loss,
  reconnect và hành vi khi tắt Host/AP; không chỉ kiểm tra RViz “có hình”.
- Quy trình hai máy và artifact nghiệm thu tuân theo
  `docs/deployment/remote-rviz2-wlan.md`; local loopback/headless không được dùng
  để đóng gate WLAN hai máy.

## 6. Kinematics, dynamics và thư viện chuyên biệt

- `KinematicsPort` là contract ổn định. PoE hiện tại là reference backend nhỏ,
  deterministic và phải tiếp tục chạy trong minimal core.
- PoE là provider mặc định/reference của v0.2.0. Pinocchio 2.6.17 là optional
  provider để load URDF, FK/Jacobian và frame placement; tích hợp sau port riêng,
  không import trực tiếp từ public API/application/ROS node.
- Kết quả Pinocchio phải được cross-check với PoE và URDF golden poses trước khi
  dùng trong command path. Frame convention, joint order và quaternion order
  phải được chuyển đổi tại boundary có test.
- IK chỉ nhận target đúng cặp frame mà provider khai báo
  (`base_link→tool0` trong model canonical); mismatch parent/child phải
  fail-fast, không được âm thầm diễn giải lại pose.
- Không duy trì hai nguồn joint limit/model. URDF/xacro và config provenance là
  nguồn; provider chỉ đọc/biên dịch model đó.
- NumPy là dependency số học nền. SciPy chỉ optional cho rotation/optimization
  khi có lợi ích được test; không thêm solver khác chỉ để thay vài phép toán nhỏ.
- MoveIt 2/OMPL thuộc ROS planning layer; simulator tương lai thuộc adapter;
  OpenCV thuộc camera adapter. Chúng không được đi ngược dependency direction
  vào domain/application.
- Mỗi dependency mới cần ADR hoặc ghi trong `PLANS.md`: capability cần thiết,
  version/Python/ROS/Jetson compatibility, ARM64 availability, license, install
  size, startup/runtime cost, fallback và test matrix.
- C++/native library không được chạy trong callback safety-critical trước khi có
  test exception boundary, thread behavior và benchmark trên thiết bị đích.
- Dynamics/torque/gravity compensation luôn bị vô hiệu trong v0.2.0. Chỉ ADR
  sau release mới được mở lại khi inertial, payload, torque limit và sign
  convention có provenance và test.

## 7. Camera

- Tên instance dựa trên phần cứng: `logitech_c922_01`.
- `role`, `hardware_serial`, `device.by_id` là field riêng; không nhét vào tên.
- Profile camera thật của v0.2.0 bắt buộc dùng identity/path ổn định như
  `/dev/v4l/by-id`; không fallback ngầm sang `/dev/video*`.
- OpenCV là optional dependency; control-only deployment không được kéo camera stack.
- Capture adapter phải enforce width/height/pixel-format đã cấu hình và giữ
  sequence tăng đơn điệu qua reopen; reconnect không được biến frame mới thành
  frame cũ trong latest-frame queue.
- Unit test camera dùng mock backend, không phụ thuộc thiết bị thật.

## 8. Quy trình thay đổi

1. Đọc `PLANS.md`, docs thiết kế và file lân cận.
2. Viết/điều chỉnh test trước hoặc cùng lúc với implementation.
3. Thay đổi nhỏ, có một mục tiêu; không refactor ngoài scope.
4. Dùng `./tools/bootstrap_core.sh` một lần và chạy `./tools/test_core.sh`.
5. Với ROS Foxy, dùng `./tools/bootstrap_ros.sh` và `./tools/test_ros.sh`; không
   trộn pytest/plugin ROS vào `.venv-core`.
6. Chạy `./tools/test_all.sh`; script không được ghi user site hoặc tự xóa cache.
7. Cập nhật `CHANGELOG.md`, docs và version nếu tạo release.
8. Chạy `.venv-core/bin/python tools/verify_release.py`.
9. Không tuyên bố robot thật/ROS graph đã đạt nếu chưa chạy trên môi trường đó.

## 9. Definition of done

Một thay đổi chỉ hoàn tất khi:

- test mới và regression test đều đạt;
- Ruff và mypy không có error; coverage toàn core cuối P7 tối thiểu 85%, từng
  nhóm config/safety/runtime/adapters tối thiểu 90%;
- dependency direction không bị phá;
- log/error có đủ context để debug;
- config có owner rõ và được validate fail-fast;
- file release được phân loại `active`/`optional`; generated/legacy/scaffold
  không được track hoặc đóng gói;
- README/changelog/plans phản ánh đúng trạng thái;
- generated cache/log/build artifacts không nằm trong release ZIP.
- thay đổi ROS network có WLAN loss/reconnect test và ghi RMW/QoS/domain;
- thay đổi kinematics/provider có golden cross-backend test và dependency
  compatibility report trên môi trường đích.
