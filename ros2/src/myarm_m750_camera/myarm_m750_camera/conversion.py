"""Direct NumPy/core to ROS message conversion without cv_bridge."""

from __future__ import annotations

import array
from typing import Tuple

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from myarm_m750_core.domain.camera import CameraConfig, CameraFrame
from sensor_msgs.msg import CameraInfo, Image


def wall_time_message(wall_time_s: float) -> Time:
    """Convert non-negative wall seconds to a normalized ROS timestamp."""
    seconds = max(0.0, float(wall_time_s))
    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=whole, nanosec=nanoseconds)


def image_message(frame: CameraFrame, frame_id: str) -> Image:
    """Copy one contiguous ndarray into a standard sensor_msgs/Image."""
    image = frame.image
    if image.dtype != np.uint8:
        raise ValueError(
            f"{frame.encoding} requires uint8 image data; got {image.dtype}."
        )
    if frame.encoding == "mono8":
        if image.ndim != 2:
            raise ValueError("mono8 requires a two-dimensional ndarray.")
    elif frame.encoding in ("bgr8", "rgb8"):
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(
                f"{frame.encoding} requires an HxWx3 ndarray."
            )
    else:
        raise ValueError(
            "Unsupported raw image encoding; expected mono8, bgr8, or rgb8."
        )
    if not image.flags["C_CONTIGUOUS"]:
        image = image.copy(order="C")
    message = Image()
    message.header.stamp = wall_time_message(frame.observation_wall_time_s)
    message.header.frame_id = frame_id
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = frame.encoding
    message.is_bigendian = False
    message.step = int(image.strides[0])
    # Foxy validates generic byte sequences element-by-element.  Supplying the
    # generated message's native uint8 container keeps the copy in C.
    message.data = array.array("B", image.tobytes())
    return message


def camera_info_message(config: CameraConfig, stamp: Time) -> CameraInfo:
    """Convert validated calibration into CameraInfo."""
    calibration = config.calibration
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = config.optical_frame
    message.width = calibration.image_width_px
    message.height = calibration.image_height_px
    message.distortion_model = calibration.distortion_model
    message.d = list(calibration.distortion_coefficients)
    message.k = list(calibration.camera_matrix)
    message.r = list(calibration.rectification_matrix)
    message.p = list(calibration.projection_matrix)
    return message


def static_transforms(config: CameraConfig) -> Tuple[TransformStamped, ...]:
    """Return measured mounting TF and REP-103 optical-frame rotation."""
    stamp = Time()
    mounting = TransformStamped()
    mounting.header.stamp = stamp
    mounting.header.frame_id = config.extrinsics.parent_frame
    mounting.child_frame_id = config.extrinsics.child_frame
    translation = config.extrinsics.translation_m
    quaternion = config.extrinsics.quaternion_xyzw
    mounting.transform.translation.x = translation[0]
    mounting.transform.translation.y = translation[1]
    mounting.transform.translation.z = translation[2]
    mounting.transform.rotation.x = quaternion[0]
    mounting.transform.rotation.y = quaternion[1]
    mounting.transform.rotation.z = quaternion[2]
    mounting.transform.rotation.w = quaternion[3]

    optical = TransformStamped()
    optical.header.stamp = stamp
    optical.header.frame_id = config.camera_frame
    optical.child_frame_id = config.optical_frame
    # REP-103 camera_link (x forward, z up) to optical (z forward, x right).
    optical.transform.rotation.x = -0.5
    optical.transform.rotation.y = 0.5
    optical.transform.rotation.z = -0.5
    optical.transform.rotation.w = 0.5
    return mounting, optical
