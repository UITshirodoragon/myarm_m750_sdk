# Remote RViz2 qua WLAN — runbook v0.2.0

Tài liệu này mô tả gate hai máy của Phase 4b. Các lệnh dưới đây chưa phải bằng
chứng nghiệm thu trên WLAN thật. Chỉ đánh dấu P4b hoàn tất sau khi lưu báo cáo
JSON/CSV của một lần chạy đủ các tình huống ở cuối tài liệu.

## Contract triển khai

| Jetson/robot computer | Host PC |
| --- | --- |
| Python core, driver, safety, `robot_state_publisher`, TF, joint state, diagnostics | description/mesh, visualization, RViz2, diagnostics và network probe |
| Sở hữu duy nhất action server và serial | Không cài/chạy core hoặc driver |
| Camera tắt trong profile control-only | Remote command tắt |

Hai máy phải dùng ARM64/x86_64 Ubuntu tương thích ROS 2 Foxy, cùng
`ROS_DOMAIN_ID`, `rmw_fastrtps_cpp` và Fast DDS profile đã sinh từ contract.
Không trộn Cyclone DDS vào deployment v0.2.0. `/joint_states` và diagnostics là
reliable/depth 5; TF tĩnh và robot description là transient-local; camera, khi
được bật bằng profile riêng, là best-effort/depth 1.

Mất Host, RViz hoặc WLAN không được đổi safety state hay dừng execution cục bộ
trên Jetson. Host launch không tạo action server. Bật remote command cần ADR
riêng về authentication/VPN hoặc SROS2, arbitration, rate limit và local
E-stop; profile này chưa có trong v0.2.0.

## 1. Chuẩn bị chung

Trên cả hai máy:

1. Cài ROS 2 Foxy và `rmw_fastrtps_cpp`; đồng bộ clock bằng chrony/NTP.
2. Cấu hình firewall cho DDS/RTPS theo domain và mạng WLAN được phép.
3. Cài cùng version `myarm_m750_description`; không truyền mesh qua DDS.
4. Dùng IP tĩnh hoặc DHCP reservation. Xác nhận interface và IPv4 bằng
   `ip -br address`, clock bằng `chronyc tracking`, và route bằng `ip route`.
   Lưu nguyên output `chronyc tracking` của cả hai máy cùng thời điểm đo;
   message age ROS không được dùng thay cho clock offset.
5. Kiểm tra AP có client isolation, multicast filtering hoặc power-save gây
   mất discovery hay không.

Mọi lệnh build dưới đây bắt đầu tại repo root. Build Jetson chỉ với các package
nó sở hữu; `.venv-ros` giữ Python Core trong môi trường riêng và dùng đúng
Python 3.8/system packages của Foxy:

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
source ros2/install/setup.bash
```

Build Host từ cùng version source, không bootstrap/cài Python Core hoặc driver:

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

## 2. Tạo contract theo từng máy

Tạo staging path không commit và sao chép hai example. Dùng cùng absolute path
trên máy sinh artifact và máy chạy để `FASTRTPS_DEFAULT_PROFILES_FILE` trong
environment artifact không bị sai sau khi chép:

```bash
mkdir -p \
  /var/tmp/myarm-m750-wlan/host \
  /var/tmp/myarm-m750-wlan/jetson
cp ros2/src/myarm_m750_bringup/config/network_jetson_wlan.example.yaml \
  /var/tmp/myarm-m750-wlan/jetson/network.yaml
cp ros2/src/myarm_m750_visualization/config/network_host_wlan.example.yaml \
  /var/tmp/myarm-m750-wlan/host/network.yaml
```

Sửa hai YAML và thay đúng domain, interface, IPv4 cùng peer/server. Các địa chỉ
trong example chỉ minh họa, không phải deployment default.

Ba mode được hỗ trợ:

- `multicast`: chỉ dùng khi AP chuyển multicast ổn định;
- `peer`: sinh initial peer/unicast profile cho hai IP cố định;
- `discovery_server`: khai báo `server: IPv4:port` và vận hành Fast DDS
  Discovery Server ngoài launch này.

Sinh artifact Jetson trên Host/deployment workstation rồi chép đúng hai file
XML/env sang Jetson. Không dùng `--check-interface` ở bước này vì contract mô tả
interface của máy khác:

```bash
ros2 run myarm_m750_visualization validate_network \
  --config /var/tmp/myarm-m750-wlan/jetson/network.yaml \
  --role jetson \
  --profile-output /var/tmp/myarm-m750-wlan/jetson/fastdds.xml \
  --environment-output /var/tmp/myarm-m750-wlan/jetson/network.env \
  --json
```

Trên Jetson, đối chiếu `MYARM_M750_WLAN_INTERFACE` trong env với
`ip -br address` và xác nhận XML whitelist đúng IPv4 trước khi launch. Có thể
cài tạm visualization tooling trong image nghiệm thu và chạy lại cùng lệnh với
`--check-interface`; package này không phải runtime owner và không được chạy
RViz/network probe trên Jetson.

Trên Host, validator kiểm tra trực tiếp interface cục bộ:

```bash
ros2 run myarm_m750_visualization validate_network \
  --config /var/tmp/myarm-m750-wlan/host/network.yaml \
  --role host \
  --check-interface \
  --profile-output /var/tmp/myarm-m750-wlan/host/fastdds.xml \
  --environment-output /var/tmp/myarm-m750-wlan/host/network.env \
  --json
```

Validator phải fail khi role, RMW, domain, interface, địa chỉ, discovery hoặc
budget sai/thiếu. Không tự sửa YAML để bỏ `--check-interface`.

Chép cả ba artifact Jetson tới đúng absolute path trên Jetson:

```bash
MYARM_JETSON_SSH='user@192.168.0.110'
ssh "${MYARM_JETSON_SSH}" \
  'mkdir -p /var/tmp/myarm-m750-wlan/jetson'
scp /var/tmp/myarm-m750-wlan/jetson/network.yaml \
  /var/tmp/myarm-m750-wlan/jetson/fastdds.xml \
  /var/tmp/myarm-m750-wlan/jetson/network.env \
  "${MYARM_JETSON_SSH}:/var/tmp/myarm-m750-wlan/jetson/"
```

Thay `MYARM_JETSON_SSH` bằng account/IP thật. Sau khi chép, kiểm tra lại
interface, IP và XML trên Jetson. Không source file env dành cho vai trò khác.

## 3. Khởi động read-only trước

Đầu tiên dùng mock adapter và tắt command interfaces trên Jetson:

```bash
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

Sau khi mock discovery/TF/probe đã đạt, có thể chuyển sang **quan sát state
robot thật** mà vẫn không mở command qua ROS. Dừng launch mock, tạo profile
deployment riêng (không chạy trực tiếp file `.example.yaml`), rồi preflight:

```bash
MYARM_REAL_CONFIG=/etc/myarm-m750/robot-real.yaml
.venv-ros/bin/python -c \
  'import sys; from myarm_m750_core import RobotSessionBuilder; b = RobotSessionBuilder.from_file(sys.argv[1]); print(b.inspect_environment()); print(b.probe_hardware())' \
  "${MYARM_REAL_CONFIG}"
```

`probe_hardware()` chỉ mở serial, đọc identity/state và đóng; không gửi motion.
Abort nếu environment có issue hoặc model/firmware/serial/mapping/capability
evidence không khớp hồ sơ đã duyệt. Sau đó launch hardware read-only:

```bash
ros2 launch myarm_m750_bringup robot.launch.py \
  core_config_file:="${MYARM_REAL_CONFIG}" \
  network_environment_file:=/var/tmp/myarm-m750-wlan/jetson/network.env \
  use_real_hardware:=true \
  enable_command_interfaces:=false
```

Host sẽ nhận joint state/TF thật nhưng không được thấy action server. Remote
command không thuộc profile này và không được bật để tiện thử.

Host chỉ quan sát và ghi cả JSON lẫn CSV:

```bash
source /opt/ros/foxy/setup.bash
source ros2/install/setup.bash
unset ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY
set -a
source /var/tmp/myarm-m750-wlan/host/network.env
set +a
# Không có default: nhập signed offset từ clock report của lần nghiệm thu.
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

Dùng `headless:=true` để chạy probe không cần GUI. Trước khi chuyển sang profile
robot thật, xác nhận Host không có node `myarm_m750_driver` hoặc action server
thứ hai và Jetson vẫn publish state/diagnostics khi đóng Host.

Trong terminal Host khác, source đúng ROS setup/install và Host env như trên,
dừng ROS daemon cũ nếu nó đã được tạo với domain khác, rồi kiểm tra:

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
`enable_command_interfaces:=false`. Nếu ping được nhưng graph rỗng, kiểm tra
`ROS_DOMAIN_ID`, RMW, XML/interface/IP, firewall, AP client isolation và xác
nhận `ROS_LOCALHOST_ONLY` không bằng `1`.

`measured_clock_offset_ms` là measurement ngoài ROS, có dấu, lấy từ chrony/NTP
report; `clock_offset_source` phải chỉ rõ artifact đo. Probe xuất riêng
`clock_sync.measured_clock_offset_ms` và
`clock_sync.absolute_clock_offset_ms`. Trường
`source_stamp_skew_p99_ms` gồm cả transport age nên không phải NTP/PTP offset.
Nếu thiếu source/measurement thì WLAN budget bắt buộc fail. Launch loopback
local inject `0.0 ms` vì publisher và subscriber dùng cùng system clock; giá
trị đó không phải bằng chứng P4b.

## 4. Budget và kịch bản nghiệm thu

Budget provisional control-only:

- effective joint-state rate `>= 4.5 Hz`;
- message age p95 `<= 250 ms`, p99 `<= 500 ms`;
- maximum gap `<= 1 s`, reconnect `<= 15 s`;
- explicit absolute clock offset `<= 20 ms`;
- joint state + diagnostics bandwidth `<= 1 Mbit/s`.

Chạy ít nhất 15 phút cho từng discovery mode được chọn và lưu config, report,
ROS/Fast DDS version, interface, AP model/firmware, RSSI và clock status. Trong
mỗi lần chạy:

1. Khởi động Host sau Jetson và đo discovery.
2. Đóng/mở lại RViz và network probe.
3. Tắt/bật Host WLAN.
4. Restart AP hoặc chuyển AP theo deployment thực.
5. Gây packet loss/jitter có kiểm soát nếu môi trường test cho phép.
6. Xác nhận robot local không fault/stop vì mất observer.
7. Xác nhận report cuối có `clock_sync.available: true`,
   `budget_passed: true` và không có `budget_violations`.

Camera mặc định tắt. Gate camera WLAN riêng dùng tối đa 640×480@15 FPS và
budget provisional `<= 5 Mbit/s/camera`; không gộp kết quả đó vào budget
control-only.

## Điều kiện abort và bằng chứng cần lưu

Dừng nghiệm thu nếu có driver/action server trên Host, domain/interface không
đúng contract, clock offset vượt budget, state stale nhưng diagnostics không
cảnh báo, hoặc mất WLAN làm thay đổi robot safety state. Không chữa timestamp
bằng queue vô hạn.

Artifact nghiệm thu gồm hai YAML contract, hai Fast DDS XML, environment files
đã loại secret, JSON/CSV probe, ROS graph/QoS snapshot, log reconnect, clock
report và người vận hành. Chỉ sau khi review các artifact này mới đổi P4b từ
`[!]` sang `[x]` trong `PLANS.md`.
