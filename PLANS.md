# PLANS — Phân tích và refactor MyArm M750 SDK

## 1. Mục tiêu và cách dùng

Tài liệu này là kế hoạch đang hoạt động cho việc đưa `myarm_m750_sdk` từ
prototype v0.1.1 thành SDK điều khiển robot có contract rõ, kiểm thử tái lập và
có lộ trình ROS 2 an toàn. Mọi hạng mục tuân theo thứ tự ưu tiên trong
`AGENTS.md`:

```text
safety/correctness → traceability → readability → consistency
                   → local convention → personal preference
```

Trạng thái:

- `[x]`: đã có bằng chứng kiểm tra trong môi trường được ghi rõ;
- `[~]`: đang triển khai;
- `[ ]`: chưa bắt đầu;
- `[!]`: cần robot, camera, ROS graph hoặc dữ liệu nhà sản xuất để xác nhận.

Không đánh dấu hoàn tất bằng việc package/thư mục đã tồn tại. Một hạng mục chỉ
hoàn tất khi acceptance gate tương ứng có test, log hoặc báo cáo chạy thực tế.

## 2. Phạm vi khảo sát và baseline ngày 2026-07-25

### Repository được khảo sát

| Repository                | Vai trò trong phân tích                                                       |
| ------------------------- | ----------------------------------------------------------------------------- |
| `reBotArm_control_py`     | SDK Python tham khảo: actuator, kinematics/dynamics, controller và trajectory |
| `reBotArmController_ROS2` | ROS 2 tham khảo: driver, interface, bringup, MoveIt và demo                   |
| `myarm_m750_sdk`          | SDK cần refactor và là owner của kế hoạch này                                 |

### Bằng chứng baseline

- [x] Đọc `AGENTS.md`, README, config, public API, application services,
  adapters, runtime, tests và ROS 2 packages hiện có.
- [x] Unit test core chạy với plugin ngoài bị vô hiệu hóa:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest pycore/tests -q`:
  **30 passed**.
- [!] `./tools/test_all.sh` chưa tái lập trong workspace hạn chế vì script cài
  editable package vào user site read-only.
- [!] Pytest mặc định bị xung đột giữa plugin `launch_testing` của ROS Foxy và
  pytest đang cài; đây là lỗi môi trường/test harness, không phải unit-test
  failure của core.
- [!] `python3 tools/verify_release.py` thất bại vì yêu cầu `agent.md` và
  `plans.md`, trong khi repository hiện có `AGENTS.md` và `PLANS.md`.
- [!] Chưa có bằng chứng chạy robot thật, camera thật hoặc ROS 2 graph trong
  lần khảo sát này.
- [ ] Chưa có coverage report, static type report, latency/jitter benchmark
  chuẩn hóa hoặc hardware-in-the-loop report.

## 3. Cách reBot triển khai

### 3.1 Python SDK `reBotArm_control_py`

Luồng điều khiển chính:

```text
example / application
        ↓
RebotArmEndPose / trajectory / kinematics
        ↓
RebotArm + JointGroup
        ↓
motorbridge
        ↓
DM serial / SocketCAN / RobStride
```

Các quyết định thiết kế đáng học:

- Config YAML mô tả model, transport, motor, joint group, control mode và gain;
  cùng một API hỗ trợ DM/RS và arm/gripper.
- `JointGroup` gom joint theo chức năng và cho phép control mode độc lập
  (`MIT`, position/velocity).
- Một control loop điều phối gửi lệnh theo thứ tự group để tránh tranh chấp bus.
- Kinematics/dynamics dựa trên Pinocchio và URDF; trajectory dùng SE(3), CLIK và
  có ví dụ từ debug motor đến điều khiển pose/gravity compensation.
- Repo có đường học tương đối rõ qua example và config mẫu.

Điểm không nên sao chép nguyên trạng:

- Hardware, thread/control loop và high-level controller còn liên kết chặt;
  application có thể tiếp cận object vendor/group khá trực tiếp.
- Domain computation phụ thuộc Pinocchio/model cache và global config, làm test
  cô lập và ownership cấu hình khó hơn.
- Safety policy, error taxonomy, runtime state và capability contract chưa phải
  boundary độc lập.
- Public API chưa thể hiện đầy đủ unit, side effect, timeout và failure mode.
- Test tự động/CI và mock/fault-injection không nổi bật so với số lượng example.

### 3.2 ROS 2 SDK `reBotArmController_ROS2`

Luồng triển khai chính:

```text
launch/config
    ↓
reBotArmController (MultiThreadedExecutor)
    ├── publishers
    ├── services
    ├── actions
    └── motor passthrough
            ↓
      HardwareManager
            ↓
     reBotArm_control_py
```

Các quyết định thiết kế đáng học:

- Tách package message, controller, bringup, MoveIt config và demo.
- Dùng standard `FollowJointTrajectory` và `GripperCommand`, custom interface
  cho pose/status/low-level command chưa diễn đạt đủ bằng standard message.
- Dùng callback group và `MultiThreadedExecutor`; command dài chạy qua action,
  có feedback, cancel, timeout và arbitration.
- Có hardware config resolver theo model/channel, joint-state publisher,
  safe-home, gripper, gravity compensation và MoveIt demo.

Rủi ro kiến trúc cần tránh trong MyArm:

- `HardwareManager` lớn, giữ hardware, kinematics, dynamics, state, thread,
  homing, gripper và command arbitration trong cùng lớp.
- Một số ROS callback chứa vòng lặp, `sleep` và thao tác hardware; khó đảm bảo
  latency bounded nếu executor/callback-group config thay đổi.
- Runtime state dùng string và nhiều boolean; invalid transition chưa được type
  system/state machine bảo vệ đầy đủ.
- ROS layer truy cập thuộc tính private của SDK/controller và đồng bộ model bằng
  cache global, tạo coupling khó kiểm thử.
- Nhiều custom interface và motor passthrough làm tăng bề mặt nguy hiểm; cần
  quyền bật explicit, rate limit và safety gate nếu áp dụng.

## 4. Đánh giá `myarm_m750_sdk`

### 4.1 Nền tảng nên giữ

- Kiến trúc `api → application → domain/runtime → ports → adapters` phù hợp
  `AGENTS.md`; core không import `rclpy`.
- `RobotSession` là composition root, che vendor API khỏi application.
- Có immutable domain models, error taxonomy, explicit `DriverStateMachine`,
  `CommandResult.command_id`, mock/replay adapter và JSONL diagnostics.
- `q_ros` là canonical; offset/direction nằm trong `JointMapper` ở adapter.
- PoE FK/IK/Jacobian lấy chuỗi joint và limit từ URDF thay vì tin pose firmware.
- Baseline `MotionGuard` từng tập trung joint step/limit/workspace/singularity;
  implementation authoritative mục 6 đã thay bằng admission/validator/executor.
- Camera có port, mock/OpenCV adapter và lifecycle riêng khỏi robot.
- ROS driver dùng standard `FollowJointTrajectory`; blocking execution được đưa
  ra worker thread thay vì state timer callback.
- Pure-Python baseline hiện có 30 test đạt.

### 4.2 Khoảng trống cần refactor

| Mức | Khoảng trống                                                                                             | Hậu quả nếu chưa xử lý                                           |
| --- | -------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| P0  | Test/release harness phụ thuộc user site và auto-loaded ROS pytest plugin                                | CI/dev không tái lập; gate có thể báo sai                        |
| P0  | Release verifier và docs lệch tên `AGENTS.md`/`PLANS.md`; `agent.md` thiếu                               | Definition of done hiện không đạt                                |
| P0  | Config parser còn default ngầm cho hardware quan trọng; chưa schema/version                              | Sai port/rate/mapping có thể chỉ lộ khi runtime                  |
| P0  | Vendor adapter coi mọi SET phải có reply và quảng cáo pause/resume theo tên method                       | Capability/failure semantics có thể sai với firmware thực        |
| P0  | Stop/disconnect/fault behavior chưa có conformance test cho mọi adapter                                  | Nguy cơ SDK và firmware lệch trạng thái                          |
| P1  | `RobotSession` vừa load config, dựng kinematics, chọn adapter và cấu hình logging                        | Composition khó override/test và sẽ phình khi thêm backend       |
| P1  | Executor gửi từng waypoint qua blocking serial nhưng chưa có deadline/jitter policy và benchmark         | 5 Hz cấu hình chưa chứng minh đáp ứng robot thật                 |
| P1  | ROS action mới kiểm tra joint order/positions; tolerance, timestamp, velocity và result mapping còn mỏng | Chưa đạt contract controller chuẩn cho MoveIt                    |
| P1  | Driver node vẫn sở hữu thread/action lifecycle trực tiếp; diagnostics còn riêng lẻ                       | Shutdown/cancel/race khó chứng minh                              |
| P1  | URDF limit/inertial/velocity/effort chưa có nguồn và trạng thái xác nhận                                 | Không đủ dữ liệu an toàn cho MoveIt/dynamics                     |
| P1  | Bringup publish `myarm_m750_standalone.urdf`, marker node lại load `myarm_m750_poe_v3_2.urdf`            | Host, TF, marker và core có thể dùng hai robot model khác nhau   |
| P1  | Hai URDF được duy trì thủ công; regression hiện chỉ so sáu arm joint                                     | Tool/gripper/visual/collision có thể lệch mà test vẫn đạt        |
| P1  | `debug_host.rviz` và `robot_host.rviz` giống hệt; file MDH cũ tồn tại cạnh model PoE                     | Không rõ RViz profile nào là canonical                           |
| P1  | `model.yaml` và `visualization.yaml` chưa có consumer; marker node hard-code model, joint, topic         | Config tạo cảm giác có hiệu lực nhưng runtime không dùng         |
| P1  | Khoảng 158 MB DAE được dùng chung cho visual và collision                                                | Release nặng; collision/planning có nguy cơ chậm không cần thiết |
| P1  | `pycore/config/robot_m750.yaml` trỏ tương đối vào source tree của ROS description                        | Wheel/core standalone không thực sự sở hữu model resource        |
| P1  | Camera package, MoveIt và Gazebo phần lớn là scaffold                                                    | Tồn tại package nhưng chưa tạo capability sử dụng được           |
| P2  | `DiagnosticEvent.msg` chồng lấn với `diagnostic_msgs` và chưa chứng minh consumer                        | Tăng interface/build/network surface không cần thiết             |
| P2  | `math3d.py`, pytransform3d, spatialmath-python và Pinocchio có vùng chức năng giao nhau                  | Dễ tạo nhiều public type/convention và kết quả FK/SE(3) khác nhau |
| P2  | Chưa có actuator/group abstraction cho gripper hoặc nhiều transport                                      | Khó mở rộng như reBot nếu yêu cầu phần cứng tăng                 |
| P2  | Chưa có calibration/versioned hardware profile và migration policy                                       | Mapping thay đổi khó truy vết/rollback                           |
| P2  | Type/lint/coverage chưa là quality gate                                                                  | Debt tăng mà test chức năng không phát hiện                      |

### 4.3 Asset/model audit hiện tại

Kết quả đọc trực tiếp source ngày 2026-07-25:

- Chín DAE trong `myarm_m750_description` là chín file khác nhau, không phải
  duplicate theo hash. Tổng kích thước khoảng **157.4 MiB**, khoảng **1.38 triệu
  triangle** theo khai báo; cần giữ asset gốc nhưng tối ưu theo vai trò.
- DAE khai đơn vị nguồn `0.001 meter` và `Z_UP`; cần regression test scale/axis/
  bounding box vì importer không nhất thiết xử lý giống nhau.
- Cùng DAE chi tiết đang được dùng cho cả `<visual>` và `<collision>` trên mọi
  link có mesh. Đây là redundancy về workload, không phải redundancy về file.
- `debug_host.rviz` và `robot_host.rviz` byte-identical; launch chỉ dùng
  `robot_host.rviz`.
- `myarm_m750_mdh_v3_2.rviz`, `config/visualization.yaml` và
  `description/config/model.yaml` hiện không có runtime consumer.
- `robot.launch.py` publish model primitive `myarm_m750_standalone.urdf`, còn
  `marker_node.py` load model mesh `myarm_m750_poe_v3_2.urdf`.
- Hai URDF cùng giữ sáu joint arm nhưng standalone bỏ toàn bộ gripper. Test hiện
  chỉ so origin/axis/position limit của sáu joint, không so fixed joint, tool,
  mimic, visual, collision hay material.
- Comment canonical URDF mô tả `flange_link → tool0` có RPY bằng zero, nhưng XML
  thực tế dùng `0 -pi/2 0`; comment gọi model `v3.1` trong khi filename/config là
  `v3_2`. Đây là contract contradiction, không được “sửa cho đẹp” trước khi
  golden frame được xác nhận.
- `marker_node.py` hard-code joint list hai lần, URDF filename, topic và QoS; Host
  RViz vì vậy phải cài toàn bộ core chỉ để tính một đường marker.
- Cả hai URDF parse được ở baseline và chín DAE là XML hợp lệ; không xóa hàng
  loạt asset đang dùng chỉ vì cần tái cấu trúc.

### 4.4 Ma trận giữ/gộp/xóa/hoãn

| Thành phần                                     | Quyết định thiết kế                           | Điều kiện                                                     |
| ---------------------------------------------- | --------------------------------------------- | ------------------------------------------------------------- |
| `myarm_m750_description`                       | **Giữ và nâng thành model owner**             | Một source, manifest, provenance và contract tests            |
| `myarm_m750_visualization`                     | **Giữ, làm Host-only tối thiểu**              | Không chứa model copy; launch được từ install-space           |
| `myarm_m750_poe_v3_2.urdf` + `standalone.urdf` | **Gộp**                                       | Một xacro/source sinh mesh/primitive variant                  |
| `debug_host.rviz`                              | **Xóa**                                       | Sau khi xác nhận vẫn byte-identical với canonical profile     |
| `myarm_m750_mdh_v3_2.rviz`                     | **Review rồi gộp/xóa legacy**                 | Chỉ migrate display/view có owner và test                     |
| `model.yaml`, `visualization.yaml`             | **Wire hoặc xóa**                             | Không giữ config không có consumer                            |
| `marker_node`                                  | **Loại khỏi minimal Host; optional refactor** | Chỉ giữ overlay standard display không diễn đạt được          |
| `DiagnosticEvent.msg`/`myarm_m750_msgs`        | **Review/xóa nếu dư**                         | Ưu tiên `diagnostic_msgs`; custom msg cần unmet contract      |
| `math3d.py`                                    | **Giữ minimal PoE primitives**                | Cross-test pytransform3d; không tiếp tục nhân bản API         |
| Pinocchio provider                             | **Thêm optional**                             | Sau `KinematicsPort`, version/ARM64 gate đạt                  |
| pytransform3d tools                            | **Chọn cho optional geometry/debug profile**  | Phù hợp URDF, frame graph, mesh và offline 3D audit           |
| spatialmath-python                             | **Không thêm vào baseline hiện tại**          | Reconsider khi cần typed SE3/SO3/Twist3, symbolic hoặc RTB    |
| camera/MoveIt/Gazebo scaffold                  | **Hoãn hoặc loại khỏi base release**          | Chỉ quảng bá khi có executable + integration test             |
| reBot dynamics/centroidal/derivatives          | **Không port hiện tại**                       | MyArm chưa có inertial/payload/torque provenance              |
| reBot MIT/gravity/raw passthrough              | **Không port hiện tại**                       | Cần torque safety, watchdog, auth và HIL riêng                |
| reBot joint groups                             | **Học có điều kiện**                          | Chỉ thêm typed `ActuatorGroup` khi gripper/multi-bus thật cần |
| reBot SE(3)/CLIK/time profiles                 | **Học qua planner port**                      | Offline candidate + toàn trajectory qua admission/validator   |

## 5. Kiến trúc đích

Giữ dependency direction trong `AGENTS.md` và làm rõ composition:

```text
Python app / ROS 2 bridge / benchmark
                  ↓
          RobotSession public API
                  ↓
      command/query application services
          ↓                 ↓
   safety + runtime     kinematics domain
          ↓                 ↓
       RobotHardwarePort / KinematicsPort
                  ↓
 mock / replay / pymycobot / simulator adapters
```

Các contract bắt buộc:

1. Core dùng SI; joint order và frame có một owner.
2. `q_ros ↔ q_real` chỉ ở hardware adapter.
3. Không command nào bypass `CommandAdmission`, `TrajectoryValidator`,
   `TrajectoryExecutor` và state machine.
4. Hardware operation có timeout/deadline, taxonomy lỗi và context log.
5. Capability được adapter chứng minh bằng conformance test, không suy từ việc
   vendor object tình cờ có method.
6. ROS 2 chỉ chuyển message/lifecycle sang public API; không chứa kinematics,
   mapping hay vendor workaround.
7. Mock là mặc định; real adapter cần profile explicit và preflight.
8. Camera fault độc lập robot fault.
9. Jetson sở hữu execution/safety; Host PC mặc định chỉ chạy RViz2 và
   diagnostics. Mất WLAN/Host không được ảnh hưởng local control.
10. PoE là reference kinematics backend; Pinocchio và backend chuyên biệt khác
    chỉ tích hợp sau `KinematicsPort`, có cross-backend test và fallback.
11. Chỉ một geometry/debug library được chọn cho baseline: pytransform3d.
    Spatialmath-python là evaluation-only đến khi có ADR chứng minh unmet use
    case; type của thư viện ngoài không được trở thành public contract.

### 5.1 Chính sách biểu diễn không gian 3D

Một phép biến đổi không gian phải mang cả dữ liệu và ý nghĩa frame. Contract
canonical:

```text
T_parent_child: biến tọa độ biểu diễn trong child sang parent
translation: meter
rotation/quaternion: right-handed, Hamilton
public/ROS quaternion: [x, y, z, w]
twist/Jacobian row order: [angular, linear]
Jacobian reference frame: khai báo explicit, không dùng default của backend
```

- `RigidTransform` là value object public; matrix đồng nhất `4x4` dùng cho tính
  toán, quaternion dùng ở API/ROS boundary. Euler/RPY chỉ dùng khi đọc URDF hoặc
  nhập/xuất UI, không lưu làm trạng thái hay nội suy trajectory.
- Mọi transform có `parent_frame` và `child_frame`; tên frame trống, rotation
  không thuộc SO(3), quaternion chưa normalize hoặc transform graph có cycle
  phải fail-fast.
- Quaternion của core/ROS là XYZW nhưng pytransform3d mặc định dùng WXYZ; việc
  đổi thứ tự chỉ được nằm trong adapter/conversion có unit test. So sánh
  quaternion phải chấp nhận `q` và `-q` là cùng orientation.
- Không nhận/trả raw 6-vector twist tại public boundary. Dùng value object có
  field tên rõ `angular_velocity_rad_s`, `linear_velocity_m_s`,
  `expressed_in_frame` và `at_frame`. PoE hiện dùng `[angular, linear]`, trong
  khi spatialmath `Twist3` dùng translation trước rotation; Pinocchio ordering
  cũng phải xác minh theo exact version. Adapter chịu trách nhiệm reorder và test.
- `tf2_ros` là transform graph runtime khi chạy ROS 2. Không dùng
  `pytransform3d.TransformManager` để thay TF/DDS.
- pytransform3d dùng cho kiểm tra convention, chuyển đổi SO(3)/SE(3), transform
  graph offline, URDF inspection, calibration/debug plot và golden-test oracle.
  Thư viện này ưu tiên readability/debugging nên không nằm trong control loop
  có deadline.
- Pinocchio dùng cho robot model FK/frame Jacobian và, khi dữ liệu đã xác nhận,
  rigid-body dynamics. Pinocchio không sở hữu topic, safety policy hoặc hardware.
- PoE nội bộ giữ làm minimal/reference backend. Không tiếp tục mở rộng
  `math3d.py` thành bản sao toàn bộ pytransform3d; chỉ giữ primitive cần cho
  backend PoE và public core không dependency.
- Mỗi provider phải chuyển kết quả về cùng `RigidTransform`, joint order,
  `[angular, linear]` và reference frame trước khi rời adapter boundary.

#### 5.1.1 Quyết định pytransform3d hay spatialmath-python

| Tiêu chí MyArm M750 | pytransform3d | spatialmath-python | Kết luận |
|---|---|---|---|
| SO(3)/SE(3), quaternion, exp/log, twist | API functional trên NumPy | Class `SO3`, `SE3`, `Twist3` và hàm thấp tầng | Cả hai đáp ứng; twist convention khác core |
| Type safety cho biểu thức toán robot | Validation function, không bọc pose bằng class | Mạnh; class kiểm tra group và overload operator | Spatialmath tốt hơn |
| Fit với DTO `RigidTransform` hiện tại | Adapter NumPy mỏng | Tạo object model thứ hai cạnh DTO core và `pin.SE3` | Pytransform3d ít xâm lấn hơn |
| Frame graph | Có `TransformManager` | Không có capability tương đương trong public scope chính | Pytransform3d |
| URDF, visual/collision và mesh | Có `UrdfTransformManager`, mesh/visualizer optional | Không phải capability chính được tài liệu công bố | Pytransform3d |
| 3D debug/animation | Matplotlib/Open3D cho transform và URDF graph | Matplotlib plot/animation cho pose/frame | Pytransform3d sát asset audit hơn |
| Symbolic, spatial vector, line/plane, dual quaternion | Không phải trọng tâm | Mạnh và rộng hơn | Chưa có use case hiện tại |
| Fit với Pinocchio | Oracle/conversion/URDF audit; Pinocchio vẫn sở hữu FK | `SE3` dễ cạnh tranh với `pin.SE3` và DTO core | Pytransform3d rõ boundary hơn |
| ROS runtime | Không thay tf2/RViz | Không thay tf2/RViz; upstream cảnh báo khả năng conflict ROS2/matplotlib | Không dùng cả hai làm runtime authority |
| Control-loop performance | Upstream ưu tiên readability/debugging | Không có lợi ích đã đo so với Pinocchio/native provider | Không dùng cả hai trong deadline path |

**Quyết định hiện tại:** chọn **pytransform3d** cho extra `geometry-tools` phục
vụ URDF/frame/mesh validation và offline visualization. Không thêm
spatialmath-python vào dependency hoặc public API của release hiện tại.

Spatialmath-python chỉ được mở lại bằng ADR khi có ít nhất một nhu cầu mà stack
hiện tại không đáp ứng tốt:

- typed `SE3/SO3/Twist3` DSL cho algorithm/prototyping;
- symbolic transform derivation;
- spatial vector, dual quaternion hoặc line/plane computation;
- tích hợp trực tiếp với Robotics Toolbox for Python.

Nếu được chọn sau này, object `SE3/SO3` chỉ sống trong adapter/tool; public API
vẫn trả `RigidTransform`/NumPy. Không cài đồng thời hai geometry library trong
production chỉ để cross-check; oracle thứ hai chỉ thuộc CI/dev profile.

### 5.2 Ownership model, mesh, URDF và RViz

```text
myarm_m750_description                 myarm_m750_visualization
├── xacro/URDF source of truth         ├── launch Host RViz
├── meshes/visual                      ├── one canonical *.rviz
├── meshes/collision                   ├── optional overlay node
├── materials                          └── visualization-only parameters
├── model manifest + provenance
└── model contract tests
```

- `description` là owner duy nhất của link/joint/frame, visual/collision
  geometry, material và mesh. `visualization` không chứa bản sao URDF/mesh.
- `visualization` chỉ sở hữu cách hiển thị: RViz display, marker/overlay, camera
  panel và Host launch. Nó consume `robot_description`/TF, không tự định nghĩa
  robot model thứ hai.
- Một xacro/model source sinh các variant `full`, `lightweight` và `kinematic`;
  variant sinh ra không được sửa tay. Nếu không cần variant, xóa `standalone`
  thay vì giữ hai URDF.
- Visual mesh và collision geometry có mục tiêu khác nhau. DAE chi tiết giữ cho
  hiển thị; collision dùng primitive/convex/low-poly đã kiểm tra, không mặc định
  tái dùng visual mesh.
- Model manifest ghi version, source/license, unit/scale, axis/up convention,
  checksum, link owner và trạng thái xác nhận của limits/inertials/collision.
- Pure-Python wheel không được phụ thuộc vào đường dẫn checkout
  `ros2/src/...`. ADR phải chọn một trong hai: data package model trung lập, hoặc
  generated URDF snapshot kèm source hash trong wheel. ROS description và core
  luôn được test là cùng model fingerprint.

### 5.3 Bài học chọn lọc từ reBot

Áp dụng:

- config-driven model/hardware profile, joint group và transport capability;
- Pinocchio đọc URDF, examples theo tầng debug → FK/IK → trajectory;
- tách ROS interfaces/bringup/MoveIt/demo và dùng standard action;
- có standalone visualizer cho debug Python, nhưng không thay RViz deployment.

Không sao chép:

- global model cache, monkey-patch SDK object hoặc ROS layer truy cập private
  controller state;
- một `HardwareManager` gom hardware, dynamics, gripper, homing, thread và state;
- low-level motor passthrough bật mặc định;
- duy trì một model/config riêng cho từng demo hoặc backend;
- đưa gravity/torque control vào roadmap gần khi inertial chưa được xác nhận.

## 6. Sổ triển khai v0.2.0 — trạng thái authoritative

Phần này là nguồn trạng thái duy nhất của breaking release `0.2.0`. Phụ lục A
giữ lại roadmap khảo sát ban đầu để giải thích quyết định, nhưng checkbox trong
phụ lục không còn dùng để suy ra tiến độ.

### 6.1 Quyết định đã khóa

- Target đã chạy gate: **ARM64/aarch64, Ubuntu 20.04, Python 3.8.10, ROS 2
  Foxy, `rmw_fastrtps_cpp`**.
- Không giữ shim API/YAML/package/asset của `0.1.x`.
- Thứ tự thực hiện:

```text
P0 → P1 → P2 → P3 → P4a local
   → P5 → P6 → P4b WLAN [!]
   → P7 mock/local → camera hardware [!]
```

- PoE là provider mặc định/reference. Pinocchio `2.6.17` chỉ được chọn
  explicit; pytransform3d `3.16.0` chỉ thuộc extra `geometry-tools` và tool
  offline. Spatialmath-python, Open3D và MeshCat không thuộc release.
- Dynamics, gravity và torque bị vô hiệu. Remote command và camera stream qua
  WLAN mặc định tắt.
- Source/mock/local gate có thể đóng bằng CI/local report. Robot/camera thật,
  WLAN hai máy và dữ liệu/provenance nhà sản xuất luôn giữ `[!]`.

### 6.2 Bằng chứng quality tổng

- [x] `./tools/test_core.sh`:
  **245 passed, 2 skipped** trong minimal core (hai skip Pinocchio system),
  Ruff zero, mypy **47 files / zero
  error**, wheel build/install/import/composition smoke pass.
- [x] Coverage core **93.02%**; config **98.75%**, safety **95.93%**, runtime
  **98.18%**, adapters **95.58%**.
- [x] `.venv-core/bin/python tools/verify_release.py`: source verifier pass và
  scan cả file tracked lẫn untracked không bị ignore.
- [x] `tools/model/generate_models.py --check`: generated model deterministic.
- [x] Native Pinocchio gate trên aarch64/Foxy: **7 passed**, phiên bản
  `2.6.17`; 128 seeded configurations đạt FK/rotation/Jacobian tolerance
  `1e-9`.
- [x] `./tools/test_ros.sh`: deterministic model pass, **6/6** ROS package
  build, **11 test-result records / 0 error / 0 failure / 0 skipped**; riêng
  driver 25, camera 6 và visualization 14 test case đạt. Live install-space
  gate còn đạt lifecycle/action/feedback/cancel, canonical description/TF,
  P4a JSON/CSV 100 mẫu, bridge hai camera và MoveIt
  plan/collision/mock-execution.
- [x] Headless inspector chạy với pytransform3d `3.16.0`; minimal `.venv-core`
  không import Pinocchio, pytransform3d, OpenCV hoặc ROS.
- [!] `.venv-core/bin/python tools/verify_release.py --release-ready` cố ý
  **fail**: visual mesh hiện là **165,053,091 bytes / 1,380,504 triangles**,
  vượt budget 40 MiB / 300k triangles. Không tạo tag/archive `0.2.0` khi gate
  này còn đỏ.

### 6.3 Phase 0 — Coding contract và quality gate

- [x] Cô đọng quy tắc bắt buộc từ
  `docs/development/clean-code-clean-comment-robotics.qmd` vào `AGENTS.md`:
  Python 3.8, unit/frame, side effect, value object, error taxonomy,
  deadline/clock, workaround/comment/TODO, public docstring, callback và QoS.
- [x] Tách `.venv-core` và `.venv-ros --system-site-packages`; script không ghi
  user site và không trộn pytest ROS vào core.
- [x] Core gate gồm Ruff, mypy trên package wheel đã cài, pytest/plugin autoload
  off, coverage, compile, wheel composition smoke và source verifier.
- [x] CI core và ROS Foxy tách riêng; inventory schema có owner/consumer và
  class `active`/`optional`/`generated`/`legacy`.
- [x] Chỉ `AGENTS.md`/`PLANS.md` là path canonical/releasable; tên lowercase
  trong lịch sử khảo sát và negative migration guard không phải file release.

**Gate P0:** đạt ở source/core. Generated cache/build/log không thuộc tập file
release mà verifier duyệt.

### 6.4 Phase 1 — Config và hardware boundary

- [x] Strict `config_version: 1`, reject unknown/missing/legacy; model, joint
  order, mapping và hai fingerprint được validate trước khi adapter được mở.
- [x] `RobotSessionBuilder`/`CameraSessionBuilder` sở hữu composition và inject
  adapter, kinematics, scheduler, clock, logging; không còn
  `RobotSession.from_config()`.
- [x] Public query dùng `read_joint_state()`/`read_hardware_status()`; không
  public `pause()`/`resume()`.
- [x] Có immutable `HardwareProfile`, `FirmwareProtocolProfile`,
  `AdapterCapabilities`, `CommandContext`; deadline/freshness dùng monotonic.
- [x] `inspect_environment()` read-only; `probe_hardware()` chỉ identity/state,
  không gửi motion.
- [x] Real session và ROS activation phải probe identity trước khi nhận command.
  Capability YAML ở trạng thái khai báo; `SUPPORTED` chỉ được expose sau probe,
  method/version check và `verification_reference` không placeholder. Motion
  thật bị reject nếu stop chưa được xác minh.
- [x] Mapping fingerprint được derive từ immutable `JointMapper` và reply angle
  phải có đúng sáu phần tử; hash chỉ chứng minh software contract, còn dấu/
  offset vật lý vẫn thuộc HIL gate.
- [x] `pymycobot==4.0.5` được pin; private serial workaround có version guard và
  điều kiện xóa.
- [x] Conformance mock/replay/vendor-fake bao phủ idempotency, malformed reply,
  timeout/retry, disconnect, stop failure và capability pre/post probe.
- [!] Chờ serial by-id, firmware/version, response thật và stop semantics để
  đóng real-hardware sub-gate.

### 6.5 Phase 2 — Safety và execution

- [x] Luồng production là `CommandAdmission → TrajectoryValidator →
  AdmittedTrajectory → TrajectoryExecutor`; không còn `realtime_execution`.
- [x] Structural validation không thể tắt. Toàn trajectory được kiểm tra finite,
  canonical joint set/order, strictly increasing time, position/step,
  supplied lẫn position/time-derived velocity/acceleration, workspace
  resampling, state freshness và provenance trước write đầu tiên.
- [x] Budget bắt buộc mặc định giới hạn **1,000 trajectory point / 10,000
  workspace FK sample**; input vượt budget bị reject trước vòng FK/I/O.
- [x] State transition lưu event/reason/command ID/timestamp. Cancellation
  generation ngăn waypoint cũ sau cancel/stop đã chấp nhận; cancel ở waypoint
  cuối được tuyến tính hóa với success nên không thể cùng báo thành công.
- [x] Stop cache giữ đúng failed/rejected result; stop thất bại chuyển `FAULT`
  thay vì tạo success giả.
- [x] Watchdog bắt buộc đọc state có deadline sau mỗi waypoint; metrics gồm
  latency, jitter, overrun, stale/retry và stop latency.
- [x] Fake backend 5 Hz: zero overrun, assertion p99 operation `<160 ms` và
  p99 absolute jitter `<20 ms`; 10/20 Hz vẫn experimental.
- [x] Fault injection bao phủ stale state, write/stop error, disconnect,
  malformed/timeout, cancel race và terminal state.
- [!] Cần firmware thật để xác nhận stop latency và scheduler budget trên bus.

### 6.6 Phase 3 — ROS 2 driver

- [x] Tách converter/validator, action coordinator, lifecycle-equivalent manager,
  core facade và thin node.
- [x] Foxy dùng Trigger configure/activate/deactivate/cleanup/recover;
  diagnostics tồn tại khi inactive và mang model-contract hash.
- [x] `FollowJointTrajectory` xử lý permutation/reorder, duplicate/missing/
  unknown, dimension/finite/time/header, path/goal tolerance, desired/actual/
  error feedback, single active goal và bounded cancel.
- [x] Cancel race trước khi core publish active command và late cancel trong
  goal-tolerance settling có regression test; recovery đóng, reconnect và probe
  lại core thay vì chỉ đổi ROS state.
- [x] Explicit QoS: joint state reliable depth 5 @5 Hz; diagnostics reliable
  depth 5 @1 Hz; static description/TF transient-local; camera best-effort
  depth 1.
- [x] Default `use_real_hardware=false`,
  `enable_command_interfaces=false`; intent adapter/cờ lệch nhau fail activation.
- [x] Chỉ dùng standard action/message/diagnostics; `myarm_m750_msgs` đã xóa.
- [x] Live mock graph trên install-space: action kết thúc `CANCELED` (status 5),
  có desired/actual/error feedback, vẫn publish 4 joint-state update và 1
  diagnostic trong lúc action chạy; lifecycle đóng ở `unconfigured`.
  `/robot_description` khớp SHA256
  `9ad2c027afc83082dad3903ea82ba5a1ff5b89deb16f1dee177054f08561c355`,
  đủ 8 dynamic TF edge và 2 static TF edge canonical với QoS đã khóa.
- [!] Robot thật và action execution qua serial chưa được nghiệm thu.

### 6.7 Phase 4 — Remote RViz2 qua WLAN

- [x] P4a local/headless: Jetson role sở hữu driver/safety/state publisher/TF/
  diagnostics; Host launch chỉ description, RViz, diagnostics và network probe,
  không tạo driver hoặc action server.
- [x] Khóa Fast DDS; có local multicast và generator peer/unicast profile.
  Source verifier reject khi local Jetson/Host lệch RMW, domain hoặc interface.
- [x] Network probe JSON/CSV phát hiện cả stream silence; validated network
  contract là owner duy nhất của budget. Clock sync có measurement/source rõ;
  source-stamp skew không bị gọi sai là NTP clock offset.
- [x] Local loopback report cuối từ `./tools/test_ros.sh` chỉ đánh giá sau tối
  thiểu **100 joint-state sample**: effective rate **4.992 Hz**, p95/p99 age
  **3.064/6.366 ms**, max gap **0.202 s**, bandwidth **0.01395 Mbit/s**,
  clock `local_loopback_same_clock`/**0 ms**; đạt toàn bộ budget provisional
  mà vẫn giữ startup gap/reconnect trong report.
- [x] Runbook hai máy:
  `docs/deployment/remote-rviz2-wlan.md`; `README.md` có quick-start peer,
  cài đặt theo role, preflight read-only và launch state robot thật với command
  interface vẫn tắt.
- [!] P4b Jetson ↔ Host thật sau P6: discovery/reconnect, p50/p95/p99 age,
  rate/gap, chrony/NTP offset, bandwidth và Host/AP loss chưa có report.
- [!] Remote command tiếp tục tắt đến khi có security/arbitration gate.

### 6.8 Phase 5 — Pinocchio và pytransform3d

- [x] PoE mặc định và Pinocchio optional cùng triển khai `KinematicsPort`, chỉ
  trả NumPy/core DTO. Jacobian là end-link origin trong base,
  `[angular, linear]`.
- [x] Pinocchio map joint name/index, giữ joint ngoài arm ở neutral, mỗi thread
  có `Data` riêng và reorder `[linear, angular]` tại adapter.
- [x] Chỉ fallback PoE khi Pinocchio import/native load không khả dụng; version,
  model, frame và joint mismatch fail-fast.
- [x] DLS IK dùng chung tolerance/result contract; target sai
  `base_link→tool0` bị reject trước solve; dynamics luôn disabled.
- [x] XYZW↔WXYZ, transform convention, double cover và model fingerprint có
  regression test; inspector dùng pytransform3d offline, không vào control loop.
- [x] Dependency policy/extras không chứa spatialmath-python, Open3D hoặc
  MeshCat.

### 6.9 Phase 6 — Canonical model, mesh, RViz và MoveIt

- [x] Một editable Xacro sinh deterministic `full`, `lightweight`,
  `kinematic`; giữ transform `tool0` pitch `-π/2`.
- [x] Manifest lưu source/artifact/normalized kinematic-contract hash; strict
  consumer kiểm tra hash/schema ở core và MoveIt/install-space.
- [x] Visual asset chuyển vào `meshes/visual`; collision dùng primitive, không
  tham chiếu detailed DAE. Lightweight/kinematic không kéo visual mesh.
- [x] Standalone/MDH URDF, duplicate RViz config, marker FK node và
  visualization config không consumer đã bị xóa.
- [x] Host minimal không phụ thuộc core; canonical RViz dùng RobotModel/TF và
  observability tiêu chuẩn.
- [x] MoveIt Foxy config viết thủ công gồm SRDF chain `base_link→tool0`, KDL,
  OMPL, joint limits và controller route duy nhất
  `/myarm_m750/follow_joint_trajectory`; có launch `plan_only` và
  `mock_execution`.
- [~] Automated local MoveIt gate đạt plan-only (`SUCCESS`; artifact cuối
  **47 point**, ba lần lặp liên tiếp 47–48 point), collision blocking bị reject
  và mock execution kết thúc `SUCCEEDED` qua đúng driver action. Probe chờ
  complete driver state, dùng scaling 5% phù hợp safety admission; launch parent
  sở hữu shutdown và mọi child kết thúc sạch bằng một SIGINT.
  MoveIt-level cancel **chưa đạt** và không bị trộn vào passing gate: Foxy chỉ
  xử lý cancel
  `/execute_trajectory`/`/move_action` sau khi trajectory đã terminal
  (`GOAL_TERMINATED`), nên driver không nhận cancel. Direct driver-action
  cancel đã đạt ở P3 nhưng không được dùng để đóng mục này.
- [!] Detailed visual mesh vượt release budget; còn thiếu mesh license,
  decimation/visual QA và collision geometry review.
- [!] Limits/inertial/payload chưa có manufacturer provenance.

### 6.10 Phase 7 — Camera core và ROS bridge

- [x] Tách `cameras_mock.yaml` và `cameras_real.example.yaml`; real example
  không có camera deployable hoặc placeholder có thể mở nhầm.
- [x] Có typed state/error/metrics, calibration/extrinsic validation, relative
  path resolution, monotonic acquisition time và wall observation time.
- [x] Mỗi camera có worker riêng, latest queue depth 1, timeout/backoff/reopen,
  bounded close; fault một camera độc lập camera khác và robot.
- [x] Python ROS bridge publish direct NumPy `Image`, `CameraInfo`, static TF,
  diagnostics; không bắt buộc `cv_bridge`; topic theo hardware identity.
- [x] Mock gate bao phủ một/hai camera, timeout/reconnect/overflow/shutdown,
  encoding/stride, QoS và camera-independent robot state. Live bridge gate nhận
  Image/CameraInfo từ cả hai camera, đủ hai diagnostics và bốn static-TF child;
  artifact cuối nhận **13/13 Image**, **14/15 CameraInfo** và shutdown
  **0.567 s**.
- [x] WLAN control-only mặc định không launch camera; camera profile là
  640×480@15 FPS với budget provisional 5 Mbit/s/camera.
- [!] Webcam/JetPack/OpenCV, unplug/replug, serial/by-id, calibration,
  extrinsic và WLAN camera measurement chưa có hardware report.

### 6.11 Release blockers và dữ liệu cần owner

- [!] Robot: serial by-id, firmware/version, identity response, stop semantics,
  joint mapping/limits, E-stop, payload/tool và HIL operator/checklist.
- [!] Model: nguồn/license DAE, decimation, full visual budget, conservative
  collision review, certified limit/inertial provenance.
- [!] Camera: serial/by-id, calibration/extrinsic, JetPack/OpenCV matrix,
  capture-block/unplug/replug behavior.
- [!] Network: Host PC, WLAN/AP/firewall, Fast DDS discovery mode, domain/
  interface, chrony/NTP và loss/reconnect report.
- [!] MoveIt: sửa hoặc thay executor/controller Foxy để cancel được chuyển tới
  driver trước terminal; lặp lại plan/collision/mock-execute/cancel gate.
- [!] Release engineering: `--release-ready` phải xanh rồi mới tạo artifact,
  commit/tag `0.2.0`; source verifier xanh một mình chưa đủ.

## Phụ lục A — Roadmap khảo sát ban đầu (superseded)

Các checkbox dưới đây phản ánh backlog tại thời điểm khảo sát, không phản ánh
trạng thái implementation hiện tại. Dùng sổ triển khai ở mục 6 cho mọi quyết
định release.

### Phase 0 — Khóa baseline và sửa quality gate (P0)

- [ ] Sửa `tools/test_all.sh` để chạy trong venv/workspace do repo sở hữu, không
  ghi user site; disable/chọn rõ pytest plugin.
- [ ] Đồng bộ tên `AGENTS.md`, `PLANS.md`, README, design docs và
  `verify_release.py`; quyết định loại bỏ hay tạo compatibility `agent.md`.
- [ ] Thêm CI matrix tối thiểu cho Python hỗ trợ thực tế; tách pure-core test và
  ROS test.
- [ ] Chạy `ruff`, `mypy`, pytest và coverage bằng config thống nhất; xử lý
  lint baseline trước khi biến lint thành blocking gate.
- [ ] Loại build/install/log/cache khỏi source/release và thêm regression check.
- [ ] Ghi environment manifest vào test report: Python, OS/arch, ROS distro và
  dependency versions.
- [ ] Thêm inventory check phân loại package/file là `active`, `optional`,
  `generated`, `legacy` hoặc `scaffold`; file không có consumer/owner phải bị
  xóa, nối vào runtime hoặc có issue/timebox rõ.
- [ ] Ghi baseline hash, kích thước và consumer của URDF/mesh/RViz config trước
  khi migration để có rollback và không xóa nhầm asset người dùng vừa thêm.

**Gate P0**

- Fresh clone chạy được một lệnh test trong venv sạch.
- `30+` core tests đạt, release verifier đạt, không cần ROS/robot/camera.
- Không có generated artifact trong release archive.

### Phase 1 — Khóa hardware/config contract (P0)

- [ ] Thêm `config_version` và typed schema; không dùng default ngầm cho port,
  baudrate, mapping, timeout, command rate hoặc firmware speed của profile real.
- [ ] Validate chéo joint names/order giữa config, URDF, mapper và adapter.
- [ ] Tách `RobotSessionBuilder`/factory khỏi public session để dependency
  injection không cần patch static method.
- [ ] Định nghĩa `HardwareProfile`, `FirmwareProtocolProfile` và
  `AdapterCapabilities` immutable; ghi firmware/model/serial vào diagnostics.
- [ ] Bổ sung preflight read-only: device exists/permission, import vendor,
  firmware response shape, joint count và mapping fingerprint.
- [ ] Viết adapter conformance suite dùng fake vendor cho connect idempotency,
  malformed reply, timeout, retry, stop, disconnect và unsupported capability.
- [ ] Chỉ quảng cáo pause/resume/stop sau khi xác nhận semantics trên phiên bản
  `pymycobot` và firmware mục tiêu.
- [ ] Chuẩn hóa clock: monotonic cho deadline/duration, wall/ROS time chỉ cho
  timestamp quan sát.

**Gate P1**

- Config real thiếu hoặc sai field phải fail-fast trước khi mở serial.
- Mọi adapter chạy cùng conformance suite.
- Log lỗi có operation, port/profile, attempt, timeout và command ID; không lộ
  vendor object ra public API.

### Phase 2 — Safety và runtime execution (P0/P1)

- [ ] Viết state-transition table cho connect/execute/cancel/stop/fault/recover/
  disconnect; thêm transition tests và race-oriented tests.
- [ ] Tách command admission, trajectory validation và execution scheduling.
- [ ] Thêm absolute deadline, measured cycle time, overrun count và jitter cho
  executor; không dùng `sleep` không tính thời gian đã tiêu tốn.
- [ ] Xác định stop policy: immediate vendor stop, hold-position fallback,
  timeout và transition kết quả.
- [ ] Validate toàn trajectory trước điểm đầu: finite, joint order, monotonic
  time, step, position limit, velocity/acceleration limit và workspace sampling.
- [ ] Thêm stale-state/watchdog policy; fault injection cho serial stall,
  partial write, repeated malformed reply và disconnect giữa trajectory.
- [ ] Benchmark mock/replay/vendor-fake tại 5/10/20 Hz; chỉ tăng rate khi p95/p99
  và overrun đáp ứng budget.

**Gate P2**

- Không waypoint nào được gửi trước khi toàn trajectory qua safety admission.
- Cancel/stop bounded và luôn cho trạng thái cuối xác định.
- Báo cáo benchmark có p50/p95/p99 latency, jitter, overrun và CPU.

### Phase 3 — ROS 2 driver contract (P1)

- [ ] Tách bridge conversion, action coordinator và node composition để unit
  test phần không phụ thuộc executor.
- [ ] Hoàn thiện `FollowJointTrajectory`: header timestamp, joint reorder policy,
  point dimension, monotonic time, path/goal tolerance, feedback và error code.
- [ ] Dùng callback group/executor policy được ghi tài liệu; mọi blocking core
  call nằm ngoài timer/subscription path.
- [ ] Thêm lifecycle hoặc lifecycle tương đương rõ ràng:
  configure → inactive → active → error/shutdown.
- [ ] Publish `diagnostic_msgs/DiagnosticArray`; custom diagnostic message chỉ
  giữ khi có field không thể biểu diễn chuẩn.
- [ ] Launch mặc định mock; `use_real_hardware` cần config explicit và log cảnh
  báo nổi bật.
- [ ] Thêm launch test cho mock graph, action success/cancel/reject, shutdown và
  topic/QoS/frame contract.

**Gate P3**

- ROS graph mock chạy trên distro mục tiêu và không cần serial.
- Action cancel/timeout không block state publisher.
- Core vẫn import/test được khi không có ROS 2.

### Phase 4 — Remote RViz2 Host qua WLAN (P1)

Mô hình deployment bắt buộc:

```text
Jetson / robot computer                       Host PC
RobotSession + driver + safety                RViz2 + marker + diagnostics
joint_states + TF + diagnostics ── DDS/WLAN ────────────────►
optional action command          ◄── disabled by default ────
```

- [ ] Chọn và khóa một RMW profile được hỗ trợ trên cả Jetson và Host
  (CycloneDDS hoặc Fast DDS); ghi ROS distro, RMW, `ROS_DOMAIN_ID`, interface
  WLAN và DDS config path vào runbook.
- [ ] Cấu hình discovery theo môi trường thực: multicast khi AP cho phép; có
  peer/unicast profile khi multicast bị cô lập. Thêm preflight kiểm tra route,
  firewall, interface binding, domain và type support.
- [ ] Chia launch theo vai trò:
  `robot.launch.py` trên Jetson và `rviz_host.launch.py` trên Host; ngăn Host
  khởi động hardware driver ngoài ý muốn.
- [ ] Bảo đảm Host cài cùng description/mesh và message package; không truyền
  mesh qua WLAN. Xác định owner của `robot_description`, `/tf`, `/tf_static`.
- [ ] Định nghĩa QoS matrix cho `/joint_states`, `/tf`, `/tf_static`,
  diagnostics, markers, camera và action; history/depth phải bounded.
- [ ] Đồng bộ clock bằng chrony/NTP, publish/đo clock offset và cảnh báo khi TF
  timestamp vượt budget.
- [ ] Hoàn thiện RViz profile cho RobotModel, TF, target pose, planned/executed
  trajectory, safety state, diagnostics và camera tùy chọn.
- [ ] Thêm network metrics: discovery time, DDS reconnect count, message age,
  end-to-end latency, effective rate, packet loss và camera bandwidth.
- [ ] Thiết lập bandwidth budget riêng cho state/TF, marker và camera; dùng
  image transport/compression, giới hạn resolution/FPS và tắt camera mặc định
  trong control-only deployment.
- [ ] Test degradation: Host khởi động sau Jetson, restart RViz, đổi AP, packet
  loss/jitter, tắt Host và mất WLAN. Robot local phải tiếp tục an toàn.
- [ ] Remote command từ Host là profile riêng, mặc định `false`; trước khi bật
  phải có arbitration, rate limit, timeout, namespace/domain isolation và
  security policy (VPN/SROS2 tùy deployment).
- [ ] Viết runbook hai máy gồm install, environment, DDS config, firewall,
  launch order, topic/QoS inspection và troubleshooting discovery/TF.

**Gate P4-WLAN**

- Host thấy RobotModel/TF/joint state/diagnostics ổn định sau cold start và
  reconnect mà không chạy driver phần cứng.
- Đo p95/p99 message age và effective joint-state rate đạt budget được cấu hình;
  không đánh giá chỉ bằng mắt trên RViz.
- Tắt Host hoặc AP không làm Jetson driver chuyển `FAULT`, dừng control loop
  hoặc mất local stop capability.
- Remote command vẫn tắt nếu security/arbitration gate chưa hoàn tất.

### Phase 5 — Kinematics/dynamics providers và dependency strategy (P1)

Mục tiêu là dùng thư viện chuyên biệt ở nơi chúng tạo giá trị, nhưng giữ core
nhẹ và tránh khóa kiến trúc vào một native dependency.

| Thư viện/backend                | Vai trò dự kiến                                                                      | Chính sách                                                          |
| ------------------------------- | ------------------------------------------------------------------------------------ | ------------------------------------------------------------------- |
| PoE hiện tại                    | Reference FK/Jacobian/IK tối thiểu                                                   | Giữ trong base core và golden regression                            |
| Pinocchio                       | FK/Jacobian/frame/dynamics/gravity hiệu năng cao                                     | Optional `kinematics-pinocchio`; provider sau port                  |
| pytransform3d                   | SO(3)/SE(3), convention checks, transform graph, URDF inspection và offline 3D debug | Chọn cho optional `geometry-tools`; không chạy command loop         |
| spatialmath-python              | Type-safe SO3/SE3/Twist, symbolic và spatial-vector prototyping                      | Không chọn baseline; evaluation-only khi có unmet use case          |
| NumPy                           | Vector/matrix nền                                                                    | Base dependency                                                     |
| SciPy                           | Solver/optimization phụ trợ                                                          | Transitive/optional; không tạo convention transform thứ ba          |
| tf2_ros + robot_state_publisher | Runtime transform graph trong ROS 2                                                  | ROS-only, là nguồn TF cho RViz                                      |
| MoveIt 2 + OMPL                 | Planning/collision trong ROS 2                                                       | ROS dependency qua apt/rosdep, không pip core                       |
| MuJoCo/Gazebo                   | Simulation/HIL substitute                                                            | Adapter profile riêng                                               |
| OpenCV/image_transport          | Camera và WLAN image stream                                                          | Camera/ROS optional profile                                         |
| MeshCat                         | Standalone Python/web debug như reBot                                                | Chỉ đánh giá sau khi use case không được RViz/pytransform3d đáp ứng |
| trimesh/Open3D                  | Mesh validation hoặc interactive point-cloud/geometry                                | Dev-only và chỉ thêm theo capability có test                        |

- [ ] Viết ADR lựa chọn Pinocchio: use case, API boundary, license, supported
  version, Python/ROS compatibility, ARM64/Jetson installation và fallback PoE.
- [ ] Viết ADR spatial stack nêu rõ vì sao dùng pytransform3d, ranh giới với
  spatialmath-python, `math3d.py`, Pinocchio, tf2 và RViz; không cho nhiều thư
  viện cùng sở hữu FK hoặc public pose type.
- [ ] ADR dùng weighted criteria: URDF/frame graph 30%, convention audit 20%,
  NumPy/PoE/Pinocchio interop 20%, headless/ARM64/Python 3.8 15%, API leakage/
  dependency cost 10%, license/maintenance 5%. Kết luận hiện tại là
  pytransform3d thắng theo use case, spatialmath bị reject-for-now chứ không cấm.
- [ ] Làm time-boxed comparison spike bằng cùng dataset: identity/inverse/
  compose, quaternion round-trip, exp/log, batch trajectory, headless plot,
  import/startup/memory và Python 3.8 install. Ghi kết quả nhưng không merge cả
  hai dependency vào production.
- [ ] Xác nhận spatialmath-python chỉ có lợi thế đủ lớn nếu typed `SE3/Twist3`,
  symbolic hoặc spatial-vector use case xuất hiện; warning ROS2/matplotlib và
  transitive dependency phải được tái hiện trên image Foxy thực.
- [ ] Không cài meta-extra `pytransform3d[all]` vào production image; tách
  geometry/mesh/plot/Open3D dependency đúng use case để Jetson headless không
  kéo GUI stack.
- [ ] Thêm `PinocchioKinematicsAdapter` hoặc provider tương đương triển khai
  `KinematicsPort`; lazy import và lỗi dependency có hướng dẫn rõ.
- [ ] Cache Pinocchio `Model` theo model fingerprint nhưng cấp `Data` riêng cho
  từng worker/thread; không chia sẻ mutable computation buffer giữa callback.
- [ ] Thêm adapter chuyển đổi core XYZW ↔ pytransform3d WXYZ, `T_parent_child`
  ↔ convention của thư viện và ROS Pose/Transform; test inverse, compose,
  round-trip, active/passive, pre/post composition, quaternion double-cover và
  near-singularity.
- [ ] Thêm convention test cho twist/adjoint/Jacobian: angular-linear order,
  body/spatial/reference frame, point gắn twist và finite-difference. Nếu chạy
  spatialmath evaluation, reorder `[linear, angular]` phải explicit.
- [ ] Dùng pytransform3d `TransformManager`/`UrdfTransformManager` trong tool
  offline để kiểm tra frame graph, URDF geometry và tạo hình/animation debug;
  không publish TF hoặc gửi command từ tool này.
- [ ] Tạo `tools/model/inspect_model.py` headless mặc định: in frame tree,
  link/joint count, disconnected/cycle report, mesh resolution, bounding box và
  golden pose; GUI/Matplotlib/Open3D là flag optional.
- [ ] Không để `RobotSession` hard-code `PoeKinematics`; chọn backend từ
  versioned config qua factory, mặc định `poe` cho minimal/mock profile.
- [ ] Load cùng URDF, joint order, base/end frames và limit; không tạo Pinocchio
  model/config source thứ hai.
- [ ] Tạo golden dataset gồm home, random valid configurations, gần joint limit,
  near-singularity và target IK reachable/unreachable.
- [ ] Cross-check PoE ↔ Pinocchio ↔ robot_state_publisher/MoveIt:
  pose, Jacobian, frame convention, quaternion order và singularity metric.
- [ ] Cross-check transform primitives nội bộ ↔ pytransform3d trên property-based
  dataset SO(3)/SE(3); pytransform3d là oracle debug, không phải lý do để xóa
  reference implementation trước khi migration đạt.
- [ ] Chạy spatialmath một lần trên cùng compose/inverse/exp/log/interpolation/
  twist dataset để lưu bằng chứng ADR; không giữ nó trong regression dependency
  sau khi quyết định đã được khóa.
- [ ] Benchmark startup, FK/Jacobian/IK latency, memory và native exception
  behavior trên x86_64 Host và ARM64 Jetson.
- [ ] Pin version và distribution source theo platform; ưu tiên ROS binary khi
  distro/architecture thật sự cung cấp. Với Python 3.8/Foxy/ARM64 hiện tại phải
  có compatibility spike, không cài `latest` hoặc build source ad-hoc trên robot.
- [ ] Lưu lockfile/SBOM/license và smoke-test import cho từng optional profile;
  native dependency lỗi không được làm base core hoặc ROS driver mất khả năng
  startup với backend PoE.
- [ ] Chỉ bật Pinocchio dynamics/gravity sau khi inertial/payload/torque sign có
  provenance; output dynamics không được tự động đi vào torque command.
- [ ] Tách dependency profiles: `base`, `kinematics-pinocchio`, `geometry-tools`,
  `camera`, `simulation`, `serial`, `ros2`; lock/test versions theo deployment
  matrix.
- [ ] Đánh giá SciPy cho SE(3)/rotation/optimizer và giữ implementation nội bộ
  nếu dependency cost lớn hơn lợi ích đo được.

**Gate P5-LIB**

- Cài base core không kéo Pinocchio/pytransform3d/spatialmath-python/SciPy/ROS/
  OpenCV/simulator.
- Cùng golden input, các backend đạt tolerance translation/orientation/Jacobian
  được ghi bằng số; mismatch frame/order fail-fast.
- Core↔pytransform3d↔ROS conversion round-trip đạt tolerance, gồm XYZW/WXYZ và
  quaternion `q == -q`; tool inspection chạy được headless trong CI.
- Dependency graph/release không chứa spatialmath-python trừ khi ADR mới chứng
  minh unmet use case và có migration/removal plan cho API trùng lặp.
- Không type `pytransform3d`, `spatialmath` hoặc `pinocchio.SE3/Motion` nào thoát
  khỏi adapter/tool vào domain, public API, config hay ROS message.
- Pinocchio import/install failure có fallback PoE rõ ràng cho non-dynamics use
  case, không làm mock/control-only deployment hỏng.
- Có compatibility report trên Python 3.8/Foxy hiện tại hoặc quyết định migration
  có ADR; không giả định wheel ARM64 tồn tại.
- Mỗi concurrent worker có Pinocchio `Data` riêng và đạt determinism/race test.

### Phase 6 — Model/mesh/RViz consolidation và MoveIt 2 (P1)

- [ ] Chốt canonical model owner và migration ADR trước khi di chuyển asset;
  không sửa đồng thời kinematic origin và mesh origin trong một commit.
- [ ] Chuyển model sang một nguồn xacro/URDF canonical; thay
  `myarm_m750_standalone.urdf` bằng variant được generate hoặc xóa nếu không còn
  use case. Mở rộng consistency test tới fixed joint, tool0, gripper, mimic,
  visual, collision và model fingerprint.
- [ ] Cho `robot.launch.py`, pycore model resolver, marker/overlay, Pinocchio,
  MoveIt và test cùng consume một rendered-model artifact/hash.
- [ ] Di chuyển asset theo ownership:
  `description/meshes/visual`, `description/meshes/collision`,
  `description/urdf|xacro`; không đặt URDF/mesh trong visualization.
- [ ] Tạo mesh manifest gồm checksum, byte size, unit/scale, coordinate axis,
  source/license, link và intended use. Kiểm tra broken `package://` URI,
  bounding box bất thường, NaN/degenerate geometry và orphan mesh.
- [ ] Tạo collision geometry đơn giản hóa; đặt polygon/size budget và benchmark
  collision load/check. Không dùng mặc định cùng DAE hàng chục MB cho visual và
  collision.
- [ ] Chỉ giữ một RViz profile canonical `robot_host.rviz`. Vì
  `debug_host.rviz` hiện giống hệt nên gộp/xóa; review file
  `myarm_m750_mdh_v3_2.rviz`, migrate display còn giá trị rồi xóa/đổi tên legacy.
- [ ] `model.yaml` và `visualization.yaml` phải được schema-load bởi launch/tool
  và có test, hoặc xóa. Không giữ config “documentation-only” trong runtime path.
- [ ] Refactor `marker_node`: không hard-code URDF filename, sáu joint names,
  absolute topics hay QoS; inject model manifest/provider và namespace.
- [ ] Chỉ giữ custom marker cho target/workspace/safety/debug overlay không được
  RViz/MoveIt standard display diễn đạt. Planned trajectory trùng
  `DisplayTrajectory` phải dùng standard display hoặc có lý do đo được.
- [ ] Giải quyết core standalone model resource: không để installed config trỏ
  vào `../../ros2/src`; package generated snapshot/data artifact với source hash
  hoặc dùng model data package trung lập theo ADR.
- [ ] Sửa package ownership/dependency: description là data package nên không
  sở hữu runtime `robot_state_publisher`; bringup sở hữu publisher; thêm `xacro`
  và `ament_python`/runtime dependency đúng nơi. Clean install phải không dựa
  vào pip/user-site ngầm.
- [ ] Lập bảng provenance cho joint limit, velocity, effort, inertial, collision
  geometry và tool frame; field chưa xác nhận phải đánh dấu conservative.
- [ ] Hoàn thiện SRDF, planning group, virtual joint, end effector, kinematics,
  OMPL và controller mapping.
- [ ] So sánh FK/IK giữa core, robot_state_publisher và MoveIt trên golden poses.
- [ ] Chạy plan-only và mock execution trước; MoveIt không được gọi hardware
  bypass driver action/admission/executor.
- [ ] Thêm self-collision và workspace regression set.

**Gate P6-MOVEIT**

- Chỉ có một editable model source; mọi generated artifact có source hash và
  CI phát hiện drift. Core, TF, marker, Pinocchio và MoveIt báo cùng fingerprint.
- Không còn RViz config giống hệt, config không consumer hoặc mesh orphan.
- Full-mesh và lightweight model cho cùng FK/frame tree; collision asset đạt
  size/latency budget đã ghi và Host RViz resolve mọi `package://` URI.
- Fresh Host chỉ cài description + visualization + ROS dependencies vẫn chạy
  RobotModel/TF/Pose/diagnostics từ install-space khi source tree không tồn tại;
  optional kinematics overlay không được kéo core vào minimal profile.
- Golden FK sai số translation/orientation nằm trong tolerance đã ghi.
- MoveIt plan/cancel/mock execution đạt; collision và limit violation bị từ chối.
- Chưa chạy robot thật thì không đánh dấu hardware execution hoàn tất.

### Phase 7 — Camera standalone và ROS bridge (P1)

- [ ] Discovery theo `/dev/v4l/by-id`, fallback `/dev/video*` explicit.
- [ ] Thêm reconnect policy, frame timeout, dropped-frame/FPS metrics và
  calibration loader.
- [ ] Test một/hai mock camera và fault injection độc lập robot session.
- [ ] Implement ROS bridge `Image`/`CameraInfo`, TF và QoS cho WLAN.
- [ ] Xác nhận OpenCV/JetPack dependency profile trên Jetson.

**Gate P7-CAMERA**

- Camera core chạy không ROS 2; camera failure không đổi robot state sang FAULT.
- ROS bridge mất/reconnect camera vẫn giữ diagnostics và shutdown sạch.

### Phase 8 — Hardware-in-the-loop rollout (P0)

- [!] Xác nhận E-stop, vùng làm việc, tải, tool, firmware, joint direction,
  offset và conservative limit với người phụ trách robot.
- [!] Read-only state test, sau đó enable/stop, rồi single-joint low-amplitude
  theo checklist; mỗi bước có abort condition.
- [!] So sánh `q_ros`, feedback firmware, RViz và software FK ở golden poses.
- [!] Chạy trajectory tốc độ/tầm nhỏ, cancel, serial-loss và recovery test.
- [!] Chỉ sau các gate trên mới thử MoveIt execution, gripper hoặc workload.

**Gate P8-HIL**

- Có biên bản HIL gồm config hash, firmware/vendor SDK version, operator,
  timestamp, test result, log và rollback.
- Sai mapping, stale state, timeout hoặc stop không bounded là điều kiện dừng,
  không được “tune tiếp” trong cùng run.

### Phase 9 — Mở rộng sau khi có nhu cầu và dữ liệu (P2)

- [ ] Gripper/actuator-group port chỉ thêm khi hardware contract đã xác nhận.
- [ ] Khi có Cartesian use case, thêm `CartesianTrajectoryPlannerPort` học từ
  reBot: SE(3) geodesic, minimum-jerk/trapezoid time law, DLS/CLIK, null-space
  joint-limit objective và planner metrics. Planner chỉ tạo candidate; toàn bộ
  sampled joint trajectory vẫn qua admission/validator trước execution.
- [ ] Dùng một typed `IkSolverOptions`/`TrajectoryOptions` làm owner tolerance,
  damping, iteration và sampling; không lặp options giữa kinematics/trajectory.
- [ ] SocketCAN/motorbridge adapter chỉ thêm sau ADR về transport và arbitration.
- [ ] Gravity compensation/torque control chỉ làm khi inertial, torque limit và
  torque feedback được xác nhận.
- [ ] Không port centroidal dynamics, analytical derivatives hoặc energy module
  từ reBot cho fixed 6-DOF nếu chưa có use case, dữ liệu và acceptance test.
- [ ] Gazebo/MuJoCo adapter dùng cùng port/conformance suite.
- [ ] C++/`ros2_control` chỉ sau benchmark chứng minh Python là bottleneck.

### A.1 Thứ tự triển khai khuyến nghị

```text
P0 test gate
  → P1 config/hardware contract
    → P2 safety/runtime benchmark
      → P3 ROS mock integration
        → P4 remote RViz2/WLAN
          → P5 Pinocchio/provider validation
            → P6 MoveIt plan-only
              → P8 hardware rollout
```

Camera (P7) có thể chạy song song sau P0 vì lifecycle/fault độc lập. Phase 9
không nằm trên critical path của SDK điều khiển joint-space an toàn.

### A.2 Yêu cầu đầu vào còn thiếu

Các dữ liệu sau phải có owner và nguồn trước khi đóng gate hardware/MoveIt:

- phiên bản firmware MyArm M750 và `pymycobot` được hỗ trợ;
- serial device ổn định (`/dev/serial/by-id` nếu có), baudrate và timeout;
- joint zero, direction, offset và quy trình calibration;
- position/velocity/acceleration/effort limit đã xác nhận;
- semantics thực của `write_angles`, `stop`, `pause`, `resume`, power enable;
- link inertial, payload/tool, collision geometry và frame convention;
- E-stop, safe-home, workspace và recovery procedure;
- ROS distro/deployment matrix Jetson–Host và DDS/network constraints.
- RMW implementation, `ROS_DOMAIN_ID`, WLAN/AP multicast policy, firewall,
  bandwidth/latency budget và remote-command security policy;
- Pinocchio version/Python/ARM64 support target, acceptable cross-backend
  tolerance và license/dependency policy.
- Quyết định đúng cho `flange_link → tool0`, canonical model version và các
  variant gripper/tool.
- Nguồn/license của chín DAE, kích thước vật lý/bounding box kỳ vọng và budget
  triangle/file/load-time cho visual/collision.

Nếu chưa có dữ liệu, dùng giá trị conservative có nhãn `unverified`; không biến
ước lượng thành “manufacturer limit”.

### A.3 Definition of done cho từng task

Mỗi task refactor phải ghi:

```text
Task:
Owner/issue:
Problem/evidence:
In scope:
Out of scope:
Files/packages owned:
Safety risks and abort condition:
Unit tests:
Integration/HIL test:
Logs/metrics:
Acceptance gate:
Rollback plan:
Environment actually verified:
```

Task chỉ hoàn tất khi:

- regression và test mới đạt trong môi trường đã ghi;
- dependency direction trong `AGENTS.md` không bị phá;
- config fail-fast và có owner;
- error/log có đủ command/hardware/protocol context;
- docs/README/changelog/plans phản ánh đúng trạng thái;
- không tuyên bố ROS graph, camera hoặc robot thật nếu chưa chạy tương ứng.
- thay đổi visualization/network có two-machine WLAN loss/reconnect report;
- thay đổi kinematics/provider có golden cross-backend report và benchmark trên
  ít nhất các kiến trúc CPU nằm trong deployment matrix.

### A.4 Tài liệu kỹ thuật làm chuẩn

- [pytransform3d — scope và design philosophy](https://dfki-ric.github.io/pytransform3d/)
- [pytransform3d — SO(3), SE(3), frame và convention guide](https://dfki-ric.github.io/pytransform3d/user_guide/index.html)
- [pytransform3d — transformations, trajectories, graph và URDF API](https://dfki-ric.github.io/pytransform3d/api.html)
- [pytransform3d — UrdfTransformManager](https://dfki-ric.github.io/pytransform3d/_apidoc/pytransform3d.urdf.UrdfTransformManager.html)
- [spatialmath-python — official repository](https://github.com/rai-opensource/spatialmath-python)
- [spatialmath-python — SO3/SE3/Twist and class design](https://spatialmath-python.rai-inst.com/intro.html)
- [spatialmath-python — Twist3 convention](https://spatialmath-python.rai-inst.com/3d_pose_twist.html)
- [spatialmath-python — package metadata and ROS dependency warning](https://pypi.org/project/spatialmath-python/)
- [Pinocchio — official repository, algorithms và model formats](https://github.com/stack-of-tasks/pinocchio)
- [Pinocchio — installation/deployment documentation](https://stack-of-tasks.github.io/pinocchio/download.html)

Version/package availability phải được kiểm tra lại trong compatibility matrix
của release; link “latest” không phải version pin cho robot deployment.
