"""Thin ROS 2 bridge for independent camera-core workers."""

from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from myarm_m750_core import CameraSessionBuilder
from myarm_m750_core.domain.errors import CameraError
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from myarm_m750_camera.conversion import (
    camera_info_message,
    image_message,
    static_transforms,
)

IMAGE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class CameraBridge(Node):
    """Publish raw Image/CameraInfo/TF and diagnostics from camera core."""

    def __init__(self) -> None:
        super().__init__("myarm_m750_camera_bridge")
        self.declare_parameter("core_camera_config_file", "")
        self.declare_parameter("publish_rate_hz", 15.0)
        config_path = str(
            self.get_parameter("core_camera_config_file").value
        ).strip()
        if not config_path:
            raise RuntimeError("core_camera_config_file must be explicit.")
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive.")

        self._session = CameraSessionBuilder.from_file(config_path).build()
        self._last_sequences = {
            name: 0 for name in self._session.camera_names
        }
        self._image_publishers = {
            name: self.create_publisher(
                Image, f"/{name}/image_raw", IMAGE_QOS
            )
            for name in self._session.camera_names
        }
        self._info_publishers = {
            name: self.create_publisher(
                CameraInfo, f"/{name}/camera_info", RELIABLE_QOS
            )
            for name in self._session.camera_names
        }
        self._diagnostics = self.create_publisher(
            DiagnosticArray, "/diagnostics", RELIABLE_QOS
        )
        self._static_broadcaster = StaticTransformBroadcaster(self)
        transforms = [
            transform
            for name in self._session.camera_names
            for transform in static_transforms(self._session.config(name))
        ]
        self._static_broadcaster.sendTransform(transforms)
        self._session.start()
        self._frame_timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_frames
        )
        self._diagnostic_timer = self.create_timer(
            1.0, self._publish_diagnostics
        )

    def _publish_frames(self) -> None:
        for name in self._session.camera_names:
            try:
                frame = self._session.latest_frame(
                    name,
                    timeout_s=0.001,
                    after_sequence=self._last_sequences[name],
                )
            except CameraError:
                continue
            self._last_sequences[name] = frame.sequence
            image = image_message(
                frame, self._session.config(name).optical_frame
            )
            info = camera_info_message(
                self._session.config(name), image.header.stamp
            )
            self._image_publishers[name].publish(image)
            self._info_publishers[name].publish(info)

    def _publish_diagnostics(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for name in self._session.camera_names:
            state = self._session.state(name)
            metrics = self._session.metrics_snapshot(name)
            status = DiagnosticStatus()
            status.name = f"myarm_m750/camera/{name}"
            status.hardware_id = self._session.config(name).hardware_serial
            status.level = (
                DiagnosticStatus.OK
                if state.value == "streaming"
                else DiagnosticStatus.WARN
            )
            status.message = state.value
            status.values = [
                KeyValue(key="frames", value=str(metrics.frames_captured)),
                KeyValue(key="timeouts", value=str(metrics.read_timeouts)),
                KeyValue(key="errors", value=str(metrics.capture_errors)),
                KeyValue(key="reconnects", value=str(metrics.reconnect_count)),
                KeyValue(
                    key="queue_overflows",
                    value=str(metrics.queue_overflow_count),
                ),
                KeyValue(
                    key="last_frame_age_s",
                    value=f"{metrics.last_frame_age_s:.6f}",
                ),
            ]
            message.status.append(status)
        self._diagnostics.publish(message)

    def destroy_node(self) -> bool:
        """Bound workers before releasing ROS entities."""
        self._session.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the ROS 2 camera bridge."""
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
