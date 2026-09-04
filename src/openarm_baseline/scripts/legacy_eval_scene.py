#!/usr/bin/python3
"""Publish the non-robot visuals and TFs for the legacy LeRobot eval scene."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def wxyz_to_xyzw(quaternion: list[float]) -> tuple[float, float, float, float]:
    return quaternion[1], quaternion[2], quaternion[3], quaternion[0]


def rpy_to_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate_wxyz(vector: list[float], quaternion: list[float]) -> tuple[float, float, float]:
    """Rotate a vector by a ROS/Isaac wxyz quaternion."""
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    vx, vy, vz = vector
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz,
    )


def transform(parent: str, child: str, xyz: list[float], xyzw: tuple[float, float, float, float]) -> TransformStamped:
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = xyzw
    return message


class LegacyEvalScene(Node):
    def __init__(self) -> None:
        super().__init__("legacy_eval_scene")
        self.declare_parameter("config_file", "")
        config_file = Path(self.get_parameter("config_file").value)
        if not config_file.is_file():
            raise FileNotFoundError(f"legacy eval scene config not found: {config_file}")
        with config_file.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        # A scene marker is static. Transient-local durability makes the full
        # scene available when RViz subscribes after this node starts.
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "legacy_eval_scene_markers", marker_qos
        )
        self.transforms = self._make_transforms()
        self.markers = self._make_markers()
        self.tf_broadcaster.sendTransform(self.transforms)
        self.marker_publisher.publish(self.markers)
        # RViz can start after this node; republish immutable markers so the
        # reference scene does not depend on launch ordering.
        self.marker_timer = self.create_timer(1.0, self._publish_markers)

    def _publish_markers(self) -> None:
        self.marker_publisher.publish(self.markers)

    def _make_transforms(self) -> list[TransformStamped]:
        head = self.config["head_camera"]
        wrist = self.config["wrist_camera"]
        optical_rotation = rpy_to_xyzw(*self.config["optical_frame_rpy"])
        cube = self.config["cube"]
        robot = self.config["robot"]
        cube_offset = rotate_wxyz(cube["body_relative_xyz"], robot["root_quaternion_wxyz"])
        cube_world_xyz = [robot["root_xyz"][index] + cube_offset[index] for index in range(3)]
        return [
            transform(head["parent_frame"], "head_realsense_link", head["xyz"], tuple(head["quaternion_xyzw"])),
            transform("head_realsense_link", "head_realsense_optical_frame", [0.0, 0.0, 0.0], optical_rotation),
            transform(wrist["parent"], "wrist_realsense_link", wrist["xyz"], wxyz_to_xyzw(wrist["quaternion_wxyz"])),
            transform("wrist_realsense_link", "wrist_realsense_optical_frame", [0.0, 0.0, 0.0], optical_rotation),
            transform(cube["parent_frame"], cube["frame"], cube_world_xyz, wxyz_to_xyzw(cube["quaternion_wxyz"])),
        ]

    @staticmethod
    def _marker(marker_id: int, frame: str, marker_type: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        # Zero duration is intentional: the marker persists until deleted.
        return marker

    def _make_markers(self) -> MarkerArray:
        table = self.config["table"]
        cube = self.config["cube"]
        output = MarkerArray()

        visual = self._marker(0, "world", Marker.MESH_RESOURCE, "table_visual")
        visual.mesh_resource = table["visual_mesh"]
        # The OBJ has no mtllib/usemtl declaration, so use marker color.
        visual.mesh_use_embedded_materials = False
        visual.scale.x = visual.scale.y = visual.scale.z = 1.0
        visual.color.r, visual.color.g, visual.color.b, visual.color.a = 0.62, 0.45, 0.30, 1.0
        output.markers.append(visual)

        collision = self._marker(1, "world", Marker.CUBE, "table_collision_proxy")
        collision.pose.position.x, collision.pose.position.y, collision.pose.position.z = table["collision_center"]
        collision.scale.x, collision.scale.y, collision.scale.z = table["collision_size"]
        collision.color.r, collision.color.g, collision.color.b, collision.color.a = 0.2, 0.8, 0.2, 0.18
        output.markers.append(collision)

        task_cube = self._marker(2, cube["frame"], Marker.CUBE, "task_cube")
        task_cube.scale.x = task_cube.scale.y = task_cube.scale.z = cube["size"]
        task_cube.color.r, task_cube.color.g, task_cube.color.b, task_cube.color.a = cube["color_rgba"]
        output.markers.append(task_cube)

        for marker_id, frame in ((3, "head_realsense_link"), (4, "wrist_realsense_link")):
            # Simplified local camera-body visual; exact USD mesh is not needed.
            camera = self._marker(marker_id, frame, Marker.CUBE, f"{frame}_body")
            camera.scale.x, camera.scale.y, camera.scale.z = 0.090, 0.025, 0.025
            # Local Rz(+90 deg) maps the long body X axis to link +Y.  The
            # head link +Y is robot-left/right; on the wrist it stays lateral
            # to the wrist mount.  This affects markers only, never TFs.
            camera.pose.orientation.z = math.sqrt(0.5)
            camera.pose.orientation.w = math.sqrt(0.5)
            camera.color.r, camera.color.g, camera.color.b, camera.color.a = 0.12, 0.12, 0.12, 1.0
            output.markers.append(camera)

        return output


def main() -> None:
    rclpy.init()
    node = LegacyEvalScene()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        # SIGTERM may already have shut down the default context (for example,
        # when a launch system stops the process).  Avoid a second shutdown.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
