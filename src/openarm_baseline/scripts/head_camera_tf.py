#!/usr/bin/python3
"""Publish the calibrated body-mounted head-camera TF chain used by A0."""
from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def rpy_to_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def wxyz_to_xyzw(q: list[float]) -> tuple[float, float, float, float]:
    return q[1], q[2], q[3], q[0]


def make_tf(parent: str, child: str, xyz: list[float], xyzw: tuple[float, float, float, float]) -> TransformStamped:
    message = TransformStamped()
    message.header.frame_id, message.child_frame_id = parent, child
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = xyzw
    return message


class HeadCameraTf(Node):
    def __init__(self) -> None:
        super().__init__("baseline_a_head_camera_tf")
        self.declare_parameter("config_file", "")
        path = Path(self.get_parameter("config_file").value)
        if not path.is_file():
            raise FileNotFoundError(f"baseline A config not found: {path}")
        with path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        camera, wrist = config["head_camera"], config["wrist_camera"]
        optical_q = rpy_to_xyzw(*camera["optical_frame_rpy"])
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform([
            make_tf(camera["parent_frame"], camera["body_frame"], camera["body_xyz"], tuple(camera["body_quaternion_xyzw"])),
            make_tf(camera["body_frame"], camera["optical_frame"], [0.0, 0.0, 0.0], optical_q),
            make_tf(wrist["parent_frame"], wrist["body_frame"], wrist["body_xyz"], wxyz_to_xyzw(wrist["body_quaternion_wxyz"])),
            make_tf(wrist["body_frame"], wrist["optical_frame"], [0.0, 0.0, 0.0], rpy_to_xyzw(*wrist["optical_frame_rpy"])),
        ])
        self.get_logger().info(
            f"Camera TF: {camera['parent_frame']} -> {camera['body_frame']} -> {camera['optical_frame']}; "
            f"{wrist['parent_frame']} -> {wrist['body_frame']} -> {wrist['optical_frame']}; optical +Z is forward")


def main() -> None:
    rclpy.init()
    node = HeadCameraTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
