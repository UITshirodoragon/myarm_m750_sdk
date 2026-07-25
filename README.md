# MyArm M750 SDK

Lớp controller giữa application/ROS 2 và phần cứng MyArm M750, phát triển theo
nguyên tắc: **đơn giản, từng bước, test trước robot thật, log được và mở rộng qua
ports/adapters**.

> Mặc định an toàn: `pycore/config/default.yaml` sử dụng mock adapter. Không đổi
> sang cấu hình robot thật trước khi xác nhận joint mapping, limits, serial device,
> nút dừng và vùng làm việc.

## Mô hình hybrid

Quan hệ phụ thuộc là một chiều:

```text
ROS 2 deployment
    ├── ROS 2 nodes, actions, topics, TF, RViz2
    └── imports/calls Python Core

Python Core standalone
    ├── public robot API
    ├── FK/IK/Jacobian, safety, trajectory
    ├── mock/replay/vendor adapters
    └── standalone camera pipeline
```

Nói cách khác:

- ROS 2 có thể chạy cùng và phụ thuộc vào `pycore`.
- `pycore` không phụ thuộc ROS 2 và vẫn chạy được ở mức control/kinematics/camera cơ bản.
- Camera capture nằm trong core; package ROS 2 camera chỉ bridge sang message, TF và diagnostics.

## Cấu trúc release

```text
myarm_m750_sdk/
├── AGENTS.md
├── PLANS.md
├── requirements/
├── pycore/
│   ├── config/
│   ├── src/
│   │   ├── api/
│   │   ├── application/
│   │   ├── domain/
│   │   ├── ports/
│   │   ├── adapters/
│   │   ├── runtime/
│   │   ├── diagnostics/
│   │   └── resources/
│   ├── tests/
│   └── pyproject.toml
├── ros2/
│   └── src/
│       ├── myarm_m750_description/
│       ├── myarm_m750_driver/
│       ├── myarm_m750_bringup/
│       ├── myarm_m750_visualization/
│       ├── myarm_m750_camera/
│       └── myarm_m750_moveit_config/
├── examples/
├── benchmarks/
├── tools/
└── docs/
```

`pycore/src` là physical package root ngắn gọn. Khi cài bằng pip, setuptools ánh
xạ các thư mục này vào import namespace ổn định `myarm_m750_core.*`.

## Trạng thái nghiệm thu

| Hạng mục | Trạng thái |
| --- | --- |
| Ba demo core: joint mock, FK/IK và hai camera mock | `[x]` chạy local, không cần hardware |
| ROS 2 mock: driver, action, TF/model, diagnostics, camera và network probe | `[x]` local/headless |
| MoveIt plan-only, collision rejection và mock execution qua SDK driver | `[x]` local/headless |
| RViz2 qua WLAN giữa Jetson và Host | `[!]` có runbook/gate, chưa có artifact hai máy thật |
| Robot, camera và calibration thật | `[!]` chưa nghiệm thu hardware |

`[x] local/headless` không thay thế bằng chứng robot thật hoặc WLAN hai máy.
Trạng thái phase và blocker đầy đủ nằm trong [PLANS.md](PLANS.md).

## Cài đặt

Baseline được khóa cho release này là Ubuntu 20.04, Python 3.8, ROS 2 Foxy và
`rmw_fastrtps_cpp`; máy robot mục tiêu là ARM64. Tất cả lệnh bên dưới bắt đầu từ
thư mục gốc của repository:

```bash
cd /path/to/myarm_m750_sdk
python3 --version
```

Lệnh thứ hai phải báo Python 3.8. Script bootstrap sẽ dừng ngay nếu interpreter
khác version; không cài dependency vào user site.

### Python Core

Tạo môi trường phát triển `.venv-core` và chạy toàn bộ quality gate:

```bash
./tools/bootstrap_core.sh
./tools/test_core.sh
```

Bootstrap dùng constraints dành cho Python 3.8 và cài editable
`pycore[dev]`. Các capability tùy chọn được cài riêng theo đúng nhu cầu:

```bash
# Serial/vendor adapter cho profile robot thật.
.venv-core/bin/python -m pip install \
  --constraint requirements/constraints-py38.txt \
  --editable 'pycore[serial]'

# Công cụ inspect URDF/frame; không đi vào control loop.
.venv-core/bin/python -m pip install \
  --constraint requirements/constraints-py38.txt \
  --editable 'pycore[geometry-tools]'

# OpenCV camera backend trên Host x86_64.
.venv-core/bin/python -m pip install \
  --constraint requirements/constraints-py38.txt \
  --editable 'pycore[camera-host]'
```

Mock camera đã nằm trong minimal core và không cần `camera-host`. Trên Jetson,
OpenCV do JetPack/apt quản lý trong môi trường `--system-site-packages`; không
cài wheel OpenCV đè lên JetPack. `default_real.example.yaml` và
`cameras_real.example.yaml` chỉ là schema mẫu, không phải profile an toàn để
chạy trực tiếp.

### ROS 2 Foxy trên Jetson hoặc máy chạy full local gate

ROS 2, `rclpy`, message packages, MoveIt và Fast DDS do apt/`rosdep` quản lý;
không cài `rclpy` bằng pip. Tạo `.venv-ros` dùng system site packages, cài
dependency và chạy gate:

```bash
./tools/bootstrap_ros.sh
source /opt/ros/foxy/setup.bash
source .venv-ros/bin/activate
rosdep install --from-paths ros2/src --ignore-src --rosdistro foxy -r -y
./tools/test_ros.sh
```

`test_ros.sh` build vào thư mục tạm rồi xóa sau gate. Muốn chạy demo thủ công,
build một install-space bền trong `ros2/install`:

```bash
.venv-ros/bin/python -m colcon --log-base ros2/log build \
  --base-paths ros2/src \
  --build-base ros2/build \
  --install-base ros2/install \
  --symlink-install
source ros2/install/setup.bash
```

## Demo cơ bản đã kiểm tra

### 1. Joint motion với mock adapter

```bash
./tools/run_mock_demo.sh
```

Kết quả đạt khi command có trạng thái `succeeded`, joint cuối khớp target và
FK trả transform `base_link → tool0`. Demo chỉ ghi vào mock adapter.

### 2. FK, IK và execution phần mềm

```bash
.venv-core/bin/python examples/fk_ik_demo.py \
  --config pycore/config/default.yaml
```

Kết quả đạt khi dòng `IK + execution` báo `succeeded`. Demo dùng PoE provider
mặc định và không mở serial.

### 3. Hai camera mock standalone

```bash
./tools/run_camera_mock_demo.sh
```

Kết quả đạt khi cả `mock_wrist_01` và `mock_shoulder_02` phát frame
`(480, 640, 3)` với sequence tăng.

### 4. Inspect model/frame không GUI

```bash
.venv-core/bin/python tools/model/inspect_model.py \
  ros2/src/myarm_m750_description/urdf/generated/myarm_m750_lightweight.urdf
```

Sau khi cài extra `geometry-tools`, có thể kiểm tra thêm bằng
pytransform3d:

```bash
.venv-core/bin/python tools/model/inspect_model.py \
  ros2/src/myarm_m750_description/urdf/generated/myarm_m750_lightweight.urdf \
  --with-pytransform3d
```

Report hợp lệ phải có `"valid": true`, graph connected/acyclic và resolve được
`base_link → tool0`.

### 5. Robot mock + RViz2 local

Sau bước build ROS ở trên, mở hai terminal tại repo root và source cùng
install-space.

Terminal 1 — driver/TF/model an toàn, command interface tắt:

```bash
source /opt/ros/foxy/setup.bash
source .venv-ros/bin/activate
source ros2/install/setup.bash
ros2 launch myarm_m750_bringup robot_local.launch.py \
  core_config_file:="$(realpath pycore/config/default.yaml)" \
  enable_command_interfaces:=false
```

Terminal 2 — RViz2 và network probe loopback:

```bash
source /opt/ros/foxy/setup.bash
source .venv-ros/bin/activate
source ros2/install/setup.bash
ros2 launch myarm_m750_visualization rviz_host.launch.py \
  report_json_file:=/var/tmp/myarm-m750-local.json \
  report_csv_file:=/var/tmp/myarm-m750-local.csv
```

Dùng `headless:=true` ở terminal 2 nếu chỉ cần kiểm tra graph/probe. Gate tự
động tương ứng là `./tools/test_ros.sh`; gate này còn kiểm tra camera bridge và
MoveIt plan/collision/mock-execution. GUI RViz2 cần display thật, còn bằng chứng
hiện tại là local/headless.

## Remote RViz2 thật qua Fast DDS/WLAN

Đây là deployment read-only hai máy: Jetson sở hữu core, driver, safety,
`robot_state_publisher`, `/joint_states`, `/tf`, `/tf_static`,
`/robot_description` và `/diagnostics`; Host chỉ chạy visualization, RViz2 và
network probe. Không chạy driver thứ hai trên Host. Camera và remote command
mặc định tắt.

Phần dưới là quick-start cho mode `peer` với IP tĩnh/DHCP reservation. Runbook
nghiệm thu đầy đủ, multicast và Discovery Server nằm tại
[docs/deployment/remote-rviz2-wlan.md](docs/deployment/remote-rviz2-wlan.md).
Các giá trị IP/interface/domain trong file example phải được thay bằng mạng
thật; không dùng nguyên ví dụ để chạy.

### A. Build đúng package trên từng máy

Jetson, tại repo root:

```bash
./tools/bootstrap_ros.sh
source /opt/ros/foxy/setup.bash
source .venv-ros/bin/activate
rosdep install \
  --from-paths \
    ros2/src/myarm_m750_description \
    ros2/src/myarm_m750_driver \
    ros2/src/myarm_m750_bringup \
  --ignore-src --rosdistro foxy -r -y
.venv-ros/bin/python -m colcon --log-base ros2/log build \
  --base-paths ros2/src \
  --build-base ros2/build \
  --install-base ros2/install \
  --symlink-install \
  --packages-select \
    myarm_m750_description \
    myarm_m750_driver \
    myarm_m750_bringup
```

Host PC, tại cùng version source, không bootstrap/cài Python Core:

```bash
source /opt/ros/foxy/setup.bash
rosdep install \
  --from-paths \
    ros2/src/myarm_m750_description \
    ros2/src/myarm_m750_visualization \
  --ignore-src --rosdistro foxy -r -y
colcon --log-base ros2/log build \
  --base-paths ros2/src \
  --build-base ros2/build \
  --install-base ros2/install \
  --symlink-install \
  --packages-select \
    myarm_m750_description \
    myarm_m750_visualization
source ros2/install/setup.bash
```

### B. Tạo và validate DDS contract trên Host

Tạo staging path giống đường dẫn đích trên hai máy:

```bash
mkdir -p \
  /var/tmp/myarm-m750-wlan/host \
  /var/tmp/myarm-m750-wlan/jetson
cp ros2/src/myarm_m750_visualization/config/network_host_wlan.example.yaml \
  /var/tmp/myarm-m750-wlan/host/network.yaml
cp ros2/src/myarm_m750_bringup/config/network_jetson_wlan.example.yaml \
  /var/tmp/myarm-m750-wlan/jetson/network.yaml
```

Sửa hai YAML để có cùng `ros_domain_id` và `discovery.mode: peer`; mỗi
`interface_address` là IPv4 của chính máy đó và `peer_addresses` là IPv4 máy
còn lại. Lấy tên/IP thật bằng `ip -br address`. Sau đó, trên Host:

```bash
ros2 run myarm_m750_visualization validate_network \
  --config /var/tmp/myarm-m750-wlan/jetson/network.yaml \
  --role jetson \
  --profile-output /var/tmp/myarm-m750-wlan/jetson/fastdds.xml \
  --environment-output /var/tmp/myarm-m750-wlan/jetson/network.env \
  --json

ros2 run myarm_m750_visualization validate_network \
  --config /var/tmp/myarm-m750-wlan/host/network.yaml \
  --role host \
  --check-interface \
  --profile-output /var/tmp/myarm-m750-wlan/host/fastdds.xml \
  --environment-output /var/tmp/myarm-m750-wlan/host/network.env \
  --json
```

Không dùng `--check-interface` khi Host đang validate contract của Jetson.
Chép cả ba artifact Jetson tới đúng cùng absolute path trên Jetson, ví dụ:

```bash
MYARM_JETSON_SSH='user@192.168.0.110'
ssh "${MYARM_JETSON_SSH}" \
  'mkdir -p /var/tmp/myarm-m750-wlan/jetson'
scp /var/tmp/myarm-m750-wlan/jetson/network.yaml \
  /var/tmp/myarm-m750-wlan/jetson/fastdds.xml \
  /var/tmp/myarm-m750-wlan/jetson/network.env \
  "${MYARM_JETSON_SSH}:/var/tmp/myarm-m750-wlan/jetson/"
```

Trên Jetson, kiểm tra `interface_address`, `wlan_interface` và XML whitelist
khớp `ip -br address`. Hai máy phải đồng bộ clock bằng chrony/NTP, AP phải tắt
client isolation và firewall phải cho phép DDS/RTPS trên WLAN được phép. Ping
thành công chưa chứng minh DDS discovery.

### C. Launch Jetson read-only trước

Trên Jetson:

```bash
cd /path/to/myarm_m750_sdk
source /opt/ros/foxy/setup.bash
source .venv-ros/bin/activate
source ros2/install/setup.bash
unset ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY
set -a
source /var/tmp/myarm-m750-wlan/jetson/network.env
set +a
ros2 launch myarm_m750_bringup robot.launch.py \
  core_config_file:="$(realpath pycore/config/default.yaml)" \
  network_environment_file:=/var/tmp/myarm-m750-wlan/jetson/network.env \
  use_real_hardware:=false \
  enable_command_interfaces:=false
```

Bước đầu luôn dùng mock/read-only. Chỉ chuyển sang profile thật sau khi toàn bộ
identity, serial by-id, mapping, firmware và stop capability có evidence.

Để quan sát **state thật** sau khi gate mock/WLAN đã đạt, dừng launch mock,
tạo một profile deployment riêng từ schema example và điền toàn bộ identity đã
xác minh. Chạy preflight dưới đây trước; `probe_hardware()` chỉ mở serial để đọc
identity/state rồi đóng, không gửi motion:

```bash
MYARM_REAL_CONFIG=/etc/myarm-m750/robot-real.yaml
.venv-ros/bin/python -c \
  'import sys; from myarm_m750_core import RobotSessionBuilder; b = RobotSessionBuilder.from_file(sys.argv[1]); print(b.inspect_environment()); print(b.probe_hardware())' \
  "${MYARM_REAL_CONFIG}"
```

Chỉ khi preflight không có issue và identity khớp artifact đã duyệt, launch
Jetson với hardware thật nhưng vẫn khóa mọi command từ ROS:

```bash
ros2 launch myarm_m750_bringup robot.launch.py \
  core_config_file:="${MYARM_REAL_CONFIG}" \
  network_environment_file:=/var/tmp/myarm-m750-wlan/jetson/network.env \
  use_real_hardware:=true \
  enable_command_interfaces:=false
```

Lúc này Host ở bước D hiển thị joint state/TF thật qua WLAN, nhưng không có
action server. Không đổi `enable_command_interfaces` sang `true` trong profile
remote-observe.

### D. Launch Host RViz2 + probe

Trên Host, nhập giá trị clock offset có dấu từ artifact chrony/NTP của chính
lần đo rồi launch; lệnh không có giá trị mặc định:

```bash
cd /path/to/myarm_m750_sdk
source /opt/ros/foxy/setup.bash
source ros2/install/setup.bash
unset ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY
set -a
source /var/tmp/myarm-m750-wlan/host/network.env
set +a
read -r -p 'Signed Host-vs-Jetson clock offset (ms): ' \
  MYARM_CLOCK_OFFSET_MS
ros2 launch myarm_m750_visualization remote_observe.launch.py \
  network_config_file:=/var/tmp/myarm-m750-wlan/host/network.yaml \
  fastdds_profile_file:=/var/tmp/myarm-m750-wlan/host/fastdds.xml \
  check_local_interface:=true \
  clock_offset_source:=chronyc_tracking_host_vs_jetson \
  measured_clock_offset_ms:="${MYARM_CLOCK_OFFSET_MS}" \
  require_clock_offset_measurement:=true \
  report_json_file:=/var/tmp/myarm-m750-wlan/report.json \
  report_csv_file:=/var/tmp/myarm-m750-wlan/report.csv
```

Dùng `headless:=true` để chạy probe mà không mở GUI.

Trong terminal Host khác, source `network.env` như trên rồi kiểm tra graph:

```bash
ros2 daemon stop
ros2 node list
ros2 topic list
ros2 topic hz /joint_states
ros2 topic info /robot_description -v
ros2 topic info /tf_static -v
ros2 action list
```

Phải thấy state/model/TF/diagnostics từ Jetson và không có action khi
`enable_command_interfaces:=false`. Nếu ping được nhưng ROS graph rỗng, kiểm tra
lại đúng `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, XML/interface/IP, firewall,
client isolation và chắc chắn `ROS_LOCALHOST_ONLY` không bằng `1`.

### E. Nghiệm thu hai máy

Chạy ít nhất 15 phút và thực hiện mất/kết nối lại Host WLAN, restart observer,
đóng/mở RViz và tình huống AP của deployment. Trong lúc đó driver Jetson phải
tiếp tục publish và không fault/stop vì mất Host. Sau khi dừng launch bằng
Ctrl-C:

```bash
python3 -m json.tool /var/tmp/myarm-m750-wlan/report.json
```

Chỉ đạt khi report có `clock_sync.available: true`, `budget_passed: true`,
không có `budget_violations`, rate joint state `>= 4.5 Hz`, p95/p99 age
`<= 250/500 ms`, max gap `<= 1 s`, reconnect `<= 15 s`, absolute clock offset
`<= 20 ms` và control bandwidth `<= 1 Mbit/s`. Phải lưu YAML, XML, env đã loại
secret, JSON/CSV, graph/QoS snapshot, reconnect log, clock report và thông tin
AP. Cho tới khi có và review đủ artifact đó, WLAN hai máy vẫn là `[!]`.

## Public Python API

Robot:

```python
from myarm_m750_core import MotionProfile, RobotSessionBuilder

with RobotSessionBuilder.from_file("pycore/config/default.yaml").build() as robot:
    result = robot.move_joints(
        [0.20, -0.20, 0.15, 0.10, -0.10, 0.15],
        MotionProfile(duration_s=3.0),
    )
    print(result.status.value, result.command_id)
```

Camera không ROS 2:

```python
from myarm_m750_core import CameraSessionBuilder

with CameraSessionBuilder.from_file(
    "pycore/config/camera/cameras_mock.yaml"
).build() as cameras:
    frame = cameras.latest_frame("mock_wrist_01")
    print(frame.camera_name, frame.sequence, frame.image.shape)
```

## Quy ước không thay đổi

- `q_ros` là canonical coordinate cho URDF, TF, FK/IK và planner.
- Mapping firmware chỉ tại hardware adapter:

```text
q2_real = q2_ros + 10 deg
q3_real = q3_ros - 10 deg
```

- Đơn vị core dùng SI.
- `get_coords()` firmware không phải nguồn chân lý của pose.
- Mọi hardware command đi qua validation và explicit runtime state.
- Camera role không nằm trong hardware name; device ID là field riêng.

## Debug và test

- Unit test: `pycore/tests`.
- Release checks: `tools/verify_release.py`.
- JSONL log: command id, protocol/timeout context.
- Mock/replay adapter cho regression và fault injection.
- RViz2 RobotModel/TF; diagnostics và network probe chạy ở node/terminal Host.
- `AGENTS.md`: rule bắt buộc cho agent/contributor.
- `PLANS.md`: task, version plan và acceptance gate.
- `docs/release/MIGRATION_0.2.md`: mapping breaking API/YAML/package.

## Giới hạn đã biết

- ROS 2 Foxy local/headless mock graph đã được kiểm tra; WLAN hai máy, webcam/
  JetPack và robot thật chưa có bằng chứng nghiệm thu.
- Driver action cancel trực tiếp đã đạt local/headless, nhưng MoveIt Foxy hiện
  chỉ xử lý cancel `/execute_trajectory` sau khi mock trajectory kết thúc
  (`GOAL_TERMINATED`), nên Phase 6 chưa đóng MoveIt-level cancel/hardware
  execution.
- Bounded camera shutdown đã có ở worker/API, nhưng hành vi `capture.read()` khi
  V4L device stall/unplug còn phụ thuộc OpenCV/JetPack và vẫn là hardware gate.
- Bộ DAE visual chi tiết vượt budget release 40 MiB/300k triangles; variant
  lightweight dùng primitive đã sẵn sàng, còn decimation/license/visual QA là
  release blocker.
- Simulator không thuộc release hiện tại; không có package scaffold quảng bá
  capability chưa được triển khai.
- Velocity/effort/inertial values của URDF chưa phải dữ liệu nhà sản xuất đã chứng nhận.
- `firmware_speed` là vendor scalar, không phải rad/s.
- C++/`ros2_control` chưa được ưu tiên khi chưa có benchmark chứng minh bottleneck.
