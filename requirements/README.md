# Requirements profiles

Các file trong thư mục này là profile cài đặt theo use case, không phải một bộ
phụ thuộc bắt buộc duy nhất.

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements/dev.txt
```

| Profile | Dùng cho |
|---|---|
| `base.txt` | Python Core, mock, replay, FK/IK và unit test tối thiểu |
| `dev.txt` | lint, type check, coverage và test phát triển |
| `camera.txt` | generic camera profile; tránh thay JetPack OpenCV trên aarch64 |
| `camera-host.txt` | OpenCV wheel cho Host PC/x86 |
| `camera-jetson.txt` | dùng OpenCV do JetPack/apt cung cấp |
| `serial.txt` | robot thật qua vendor serial adapter |
| `simulation.txt` | MuJoCo extension |
| `ros2.txt` | nhắc rõ ROS 2 được quản lý bằng apt/rosdep, không cài `rclpy` bằng pip |
| `all.txt` | môi trường phát triển đầy đủ, không khuyến nghị cho Jetson production |

`pycore/pyproject.toml` vẫn là nguồn chính cho metadata package. Các profile ở
đây giúp tạo venv theo deployment mà không kéo phụ thuộc camera, simulation hoặc
robot thật vào môi trường tối thiểu.

Trên Jetson nên tạo venv bằng `python3 -m venv --system-site-packages .venv` để dùng bản `cv2` đi cùng JetPack, thay vì ghi đè bằng wheel PyPI.
