"""RViz target and planned-trajectory markers for Host PC debugging."""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker

from myarm_m750_core.domain.kinematics import PoeKinematics


class MyArmM750MarkerNode(Node):
    """Convert high-level targets and joint trajectories into RViz markers."""

    def __init__(self) -> None:
        super().__init__("myarm_m750_marker_node")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_link", "tool0")
        description_share = Path(
            get_package_share_directory("myarm_m750_description")
        )
        urdf_path = description_share / "urdf" / "myarm_m750_poe_v3_2.urdf"
        joint_names = (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
        )
        self._kinematics = PoeKinematics.from_urdf(
            urdf_path=urdf_path,
            base_link=str(self.get_parameter("base_frame").value),
            end_link=str(self.get_parameter("end_link").value),
            joint_names=joint_names,
        )
        self._marker_publisher = self.create_publisher(
            Marker, "/myarm_m750/debug_markers", 10
        )
        self.create_subscription(
            PoseStamped,
            "/myarm_m750/target_pose",
            self._target_pose_callback,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "/myarm_m750/planned_trajectory",
            self._trajectory_callback,
            10,
        )

    def _target_pose_callback(self, message: PoseStamped) -> None:
        marker = Marker()
        marker.header = message.header
        marker.ns = "target_pose"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = message.pose
        marker.scale.x = 0.12
        marker.scale.y = 0.025
        marker.scale.z = 0.025
        marker.color.r = 0.9
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 1.0
        self._marker_publisher.publish(marker)

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        if tuple(message.joint_names) != tuple(self._kinematics_joint_names()):
            self.get_logger().warning(
                "Ignored trajectory marker: canonical joint order mismatch."
            )
            return
        marker = Marker()
        marker.header.frame_id = str(self.get_parameter("base_frame").value)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "planned_trajectory"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.008
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 1.0
        for trajectory_point in message.points:
            if len(trajectory_point.positions) != 6:
                continue
            pose = self._kinematics.compute_fk(trajectory_point.positions)
            point = Point()
            point.x, point.y, point.z = pose.translation_m
            marker.points.append(point)
        self._marker_publisher.publish(marker)

    @staticmethod
    def _kinematics_joint_names() -> List[str]:
        return [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
        ]


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MyArmM750MarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
