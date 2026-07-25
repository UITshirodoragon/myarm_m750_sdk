from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from myarm_m750_core.adapters.camera import MockCameraAdapter, OpenCvCameraAdapter
from myarm_m750_core.adapters.kinematics.geometry_tools import (
    load_urdf_transform_manager,
)
from myarm_m750_core.domain.errors import (
    CameraCaptureError,
    CameraTimeoutError,
    KinematicsError,
)
from myarm_m750_core.runtime.config import load_camera_configs


class FakeCapture:
    def __init__(self, opened=True, frames=None):
        self.opened = opened
        self.frames = list(frames or [])
        self.settings = []

    def isOpened(self):
        return self.opened

    def set(self, key, value):
        self.settings.append((key, value))

    def read(self):
        if self.frames:
            return self.frames.pop(0)
        return False, None

    def release(self):
        self.opened = False


class FakeCv2:
    CAP_PROP_FRAME_WIDTH = 1
    CAP_PROP_FRAME_HEIGHT = 2
    CAP_PROP_FPS = 3
    COLOR_BGR2GRAY = 4
    COLOR_BGR2RGB = 5

    def __init__(self, captures):
        self.captures = (
            list(captures) if isinstance(captures, (list, tuple)) else [captures]
        )

    def VideoCapture(self, _device):
        return self.captures.pop(0)

    @staticmethod
    def cvtColor(image, conversion):
        if conversion == FakeCv2.COLOR_BGR2RGB:
            return np.ascontiguousarray(image[:, :, ::-1])
        if conversion == FakeCv2.COLOR_BGR2GRAY:
            return np.mean(image, axis=2).astype(np.uint8)
        raise AssertionError(f"Unexpected conversion: {conversion}")


def _opencv_config(repository_root):
    mock = load_camera_configs(
        str(repository_root / "pycore/config/camera/cameras_mock.yaml")
    )[0]
    return replace(
        mock,
        backend="opencv",
        device_by_id="/dev/v4l/by-id/fake-camera",
    )


def test_opencv_adapter_success_and_idempotent_close(repository_root, monkeypatch) -> None:
    config = _opencv_config(repository_root)
    first_capture = FakeCapture(
        frames=[
            (False, None),
            (
                True,
                np.zeros(
                    (config.stream.height_px, config.stream.width_px, 3),
                    dtype=np.uint8,
                ),
            ),
        ]
    )
    second_capture = FakeCapture(
        frames=[
            (
                True,
                np.zeros(
                    (config.stream.height_px, config.stream.width_px, 3),
                    dtype=np.uint8,
                ),
            )
        ]
    )
    backend = OpenCvCameraAdapter()
    fake_cv2 = FakeCv2([first_capture, second_capture])
    monkeypatch.setattr(type(backend), "_load_cv2", staticmethod(lambda: fake_cv2))
    monkeypatch.setattr(
        "myarm_m750_core.adapters.camera.opencv_capture.Path.exists",
        lambda _path: True,
    )
    backend.open(config)
    backend.open(config)
    frame = backend.read_frame(0.1)
    assert frame.sequence == 1
    assert frame.encoding == "bgr8"
    assert len(first_capture.settings) == 3
    backend.close()
    backend.open(config)
    assert backend.read_frame(0.1).sequence == 2
    assert len(second_capture.settings) == 3
    backend.close()
    backend.close()
    with pytest.raises(CameraCaptureError, match="not open"):
        backend.read_frame(0.1)


@pytest.mark.parametrize(
    ("pixel_format", "expected_shape", "expected_pixel"),
    [
        ("bgr8", (480, 640, 3), (1, 2, 3)),
        ("rgb8", (480, 640, 3), (3, 2, 1)),
        ("mono8", (480, 640), 2),
    ],
)
def test_opencv_and_mock_emit_the_configured_raw_format(
    repository_root,
    monkeypatch,
    pixel_format,
    expected_shape,
    expected_pixel,
) -> None:
    base_config = _opencv_config(repository_root)
    config = replace(
        base_config,
        stream=replace(base_config.stream, pixel_format=pixel_format),
    )
    image = np.empty((480, 640, 3), dtype=np.uint8)
    image[:, :, 0] = 1
    image[:, :, 1] = 2
    image[:, :, 2] = 3
    backend = OpenCvCameraAdapter()
    monkeypatch.setattr(
        type(backend),
        "_load_cv2",
        staticmethod(lambda: FakeCv2(FakeCapture(frames=[(True, image)]))),
    )
    monkeypatch.setattr(
        "myarm_m750_core.adapters.camera.opencv_capture.Path.exists",
        lambda _path: True,
    )
    backend.open(config)
    frame = backend.read_frame(0.1)
    assert frame.encoding == pixel_format
    assert frame.image.shape == expected_shape
    if pixel_format == "mono8":
        assert frame.image[0, 0] == expected_pixel
    else:
        assert tuple(frame.image[0, 0]) == expected_pixel

    mock = MockCameraAdapter()
    mock.open(replace(config, backend="mock", device_by_id=""))
    mock_frame = mock.read_frame(0.1)
    assert mock_frame.encoding == pixel_format
    assert mock_frame.image.shape == expected_shape


def test_opencv_adapter_rejects_missing_or_failed_device(
    repository_root, monkeypatch
) -> None:
    config = _opencv_config(repository_root)
    backend = OpenCvCameraAdapter()
    monkeypatch.setattr(
        "myarm_m750_core.adapters.camera.opencv_capture.Path.exists",
        lambda _path: False,
    )
    with pytest.raises(CameraCaptureError, match="does not exist"):
        backend.open(config)

    monkeypatch.setattr(
        "myarm_m750_core.adapters.camera.opencv_capture.Path.exists",
        lambda _path: True,
    )
    monkeypatch.setattr(
        type(backend),
        "_load_cv2",
        staticmethod(lambda: FakeCv2(FakeCapture(opened=False))),
    )
    with pytest.raises(CameraCaptureError, match="Could not open"):
        backend.open(config)


def test_opencv_adapter_timeout_and_invalid_timeout(repository_root, monkeypatch) -> None:
    config = _opencv_config(repository_root)
    backend = OpenCvCameraAdapter()
    monkeypatch.setattr(
        "myarm_m750_core.adapters.camera.opencv_capture.Path.exists",
        lambda _path: True,
    )
    monkeypatch.setattr(
        type(backend),
        "_load_cv2",
        staticmethod(lambda: FakeCv2(FakeCapture())),
    )
    backend.open(config)
    with pytest.raises(ValueError):
        backend.read_frame(0.0)
    with pytest.raises(CameraTimeoutError):
        backend.read_frame(0.001)


@pytest.mark.parametrize(
    ("image", "message"),
    [
        (np.zeros((479, 640, 3), dtype=np.uint8), "dimensions"),
        (np.zeros((480, 640, 4), dtype=np.uint8), "HxWx3"),
        (np.zeros((480, 640, 3), dtype=np.float32), "uint8"),
        ([[[0, 0, 0]]], "non-NumPy"),
    ],
)
def test_opencv_adapter_rejects_malformed_capture_frames(
    repository_root,
    monkeypatch,
    image,
    message,
) -> None:
    config = _opencv_config(repository_root)
    backend = OpenCvCameraAdapter()
    monkeypatch.setattr(
        "myarm_m750_core.adapters.camera.opencv_capture.Path.exists",
        lambda _path: True,
    )
    monkeypatch.setattr(
        type(backend),
        "_load_cv2",
        staticmethod(lambda: FakeCv2(FakeCapture(frames=[(True, image)]))),
    )
    backend.open(config)
    with pytest.raises(CameraCaptureError, match=message):
        backend.read_frame(0.1)


def test_geometry_tools_success_version_and_read_errors(
    repository_root, monkeypatch, tmp_path
) -> None:
    import myarm_m750_core.adapters.kinematics.geometry_tools as module

    class FakeManager:
        def __init__(self):
            self.loaded = ""
            self.joints = {}

        def load_urdf(self, xml, mesh_path=None, package_dir=None):
            self.loaded = (xml, mesh_path, package_dir)

        def set_joint(self, name, value):
            self.joints[name] = value

    package = SimpleNamespace(__version__="3.16.0")
    urdf_module = SimpleNamespace(UrdfTransformManager=FakeManager)

    def import_success(name):
        return package if name == "pytransform3d" else urdf_module

    monkeypatch.setattr(module.importlib, "import_module", import_success)
    urdf_path = repository_root / "pycore/src/resources/myarm_m750_kinematic.urdf"
    manager = load_urdf_transform_manager(urdf_path, {"shoulder_pan_joint": 0.25})
    assert manager.joints["shoulder_pan_joint"] == 0.25
    assert "<robot" in manager.loaded[0]

    package.__version__ = "0.0.0"
    with pytest.raises(KinematicsError, match="Unsupported"):
        load_urdf_transform_manager(urdf_path)
    package.__version__ = "3.16.0"
    with pytest.raises(KinematicsError, match="Could not read"):
        load_urdf_transform_manager(tmp_path / "missing.urdf")
