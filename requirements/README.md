# Dependency policy

`pycore/pyproject.toml` là nguồn duy nhất khai báo dependency và optional extra.
Không tạo lại các profile `.txt` lặp dependency vì chúng dễ lệch version và
platform marker.

| Extra | Dùng cho |
|---|---|
| `dev` | Ruff, mypy, pytest và coverage |
| `serial` | robot thật qua vendor serial adapter |
| `geometry-tools` | pytransform3d cho kiểm tra model/frame offline |
| `camera-host` | OpenCV wheel trên Host PC/x86 |

`constraints-py38.txt` khóa các direct dependency đã chọn cho target ARM64,
Ubuntu 20.04 và Python 3.8. File này là constraint, không phải danh sách cài đặt:

```bash
./tools/bootstrap_core.sh
.venv-core/bin/python -m pip install \
  --constraint requirements/constraints-py38.txt \
  --editable 'pycore[geometry-tools]'
```

ROS 2 Foxy, Pinocchio và OpenCV trên Jetson được cài bằng apt/rosdep/JetPack,
không bằng pip. Dùng `./tools/bootstrap_ros.sh` để tạo `.venv-ros` với
`--system-site-packages`; không cài một bản `rclpy`, Pinocchio hoặc OpenCV khác
đè lên system package.
