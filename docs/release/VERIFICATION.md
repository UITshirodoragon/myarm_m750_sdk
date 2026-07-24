# Verification report — MyArm M750 SDK v0.1.1

Ngày kiểm tra: 24/07/2026.

## Đã chạy trong môi trường đóng gói

- 30 pure-Python unit tests: **PASS**.
- Requested `pycore/src` và `ros2/src` physical layout regression tests: **PASS**.
- Parse toàn bộ YAML, XML và URDF: **PASS**.
- Đồng bộ version `0.1.1` giữa `VERSION`, `pyproject.toml`, ROS `package.xml`,
  ROS `setup.py` và Python `__version__`: **PASS**.
- Build wheel `myarm_m750_core-0.1.1-py3-none-any.whl`: **PASS**.
- Wheel chứa namespace `myarm_m750_core.*` dù physical source nằm trực tiếp dưới
  `pycore/src/api`, `domain`, `runtime`, ...: **PASS**.
- Contract URDF PoE v3.2: joint order, origin RPY, axis, limits và standalone
  RViz2 URDF: **PASS**.
- Contract mapping `[0, +10, -10, 0, 0, 0] degree` tại hardware boundary: **PASS**.
- Dependency boundary: domain/runtime/application/ports không import `rclpy`,
  `pymycobot` hoặc `cv2`: **PASS**.
- Safe startup: default robot adapter là `mock`: **PASS**.
- Camera config contract: hardware name, role, serial và stable by-id path tách
  riêng: **PASS**.
- Robot public API mock demo: **PASS**.
- Standalone `CameraSession` mock demo, không ROS 2: **PASS**.
- FK → numerical IK → mock execution smoke test: **PASS**.
- T1 mock waypoint: 20 repetitions × 2 waypoints = 40 successful commands.
- Byte-code compilation cho Core, examples và benchmarks: **PASS**.

Lệnh tái lập:

```bash
python3 -m pip install -r requirements/dev.txt
python3 -m pip install -e pycore
./tools/test_all.sh
./tools/run_mock_demo.sh
./tools/run_camera_mock_demo.sh
python3 examples/fk_ik_demo.py --config pycore/config/default.yaml
python3 benchmarks/mock_joint_waypoint.py \
  --sdk-config pycore/config/default.yaml \
  --benchmark-config benchmarks/config/t1_joint_waypoint.yaml
```

## Chưa được xác nhận trong môi trường đóng gói

- Chưa build/runtime test bằng ROS 2 Foxy thật vì môi trường tạo artifact không
  có ROS 2.
- Chưa kết nối MyArm M750 thật; không tuyên bố đã xác nhận protocol timing,
  cơ khí, vùng làm việc hoặc firmware limits thực.
- Chưa mở webcam Logitech thật bằng OpenCV; camera verification hiện dùng mock.
- Chưa nghiệm thu DDS/WLAN giữa Jetson và Host PC.
- MoveIt 2, Gazebo, ROS camera bridge và `ros2_control` vẫn là extension points.

Trình tự triển khai thực tế:

```text
unit test → mock → standalone camera test → RViz2/ROS graph
→ robot thật biên độ nhỏ trong vùng trống → benchmark thực tế
```
