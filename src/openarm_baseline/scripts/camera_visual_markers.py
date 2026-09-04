#!/usr/bin/python3
"""RViz-only markers for existing Baseline-A camera TF frames; publishes no TF."""
from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class CameraVisualMarkers(Node):
    def __init__(self):
        super().__init__("baseline_a_camera_visual_markers")
        self.declare_parameter("config_file", "")
        self.declare_parameter("publish_virtual_cube", True)
        self.declare_parameter("publish_head_optical_axis", True)
        self.declare_parameter("publish_wrist_optical_axis", True)
        with Path(self.get_parameter("config_file").value).open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.cameras = (config["head_camera"], config["wrist_camera"])
        self.cube, self.grasp = config["cube"], config["baseline_a"]["grasp"]
        self.publish_virtual_cube = self.get_parameter("publish_virtual_cube").value
        self.publish_head_optical_axis = self.get_parameter("publish_head_optical_axis").value
        self.publish_wrist_optical_axis = self.get_parameter("publish_wrist_optical_axis").value
        self.publisher = self.create_publisher(MarkerArray, "baseline_a/camera_visual_markers", 1)
        self.create_timer(.5, self.publish)

    @staticmethod
    def body(marker_id, frame, namespace):
        marker = Marker(); marker.header.frame_id = frame; marker.ns = namespace; marker.id = marker_id
        marker.type = Marker.CUBE; marker.action = Marker.ADD
        # Existing legacy scene visual: camera long local X axis shown along local Y.
        marker.pose.orientation.z = math.sqrt(.5); marker.pose.orientation.w = math.sqrt(.5)
        marker.scale.x, marker.scale.y, marker.scale.z = .090, .025, .025
        marker.color.r = marker.color.g = marker.color.b = .12; marker.color.a = 1.
        return marker

    def virtual_cube(self):
        marker = Marker(); marker.header.frame_id = "openarm_left_hand_tcp"; marker.ns = "baseline_a_virtual_grasp"; marker.id = 10
        marker.type = Marker.CUBE; marker.action = Marker.ADD
        xyz, q = self.grasp["tcp_to_cube_xyz"], self.grasp["tcp_to_cube_quat_xyzw"]
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
        marker.scale.x = marker.scale.y = marker.scale.z = self.cube["size_m"]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.cube["color_rgba"]
        return marker

    @staticmethod
    def optical_axis(marker_id, frame, namespace):
        marker = Marker(); marker.header.frame_id = frame; marker.ns = namespace; marker.id = marker_id
        marker.type = Marker.ARROW; marker.action = Marker.ADD
        marker.points = [Point(x=0., y=0., z=0.), Point(x=0., y=0., z=.15)]
        marker.scale.x, marker.scale.y = .008, .016
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .8, 1., 1.
        return marker

    def publish(self):
        head, wrist = self.cameras
        output = MarkerArray()
        output.markers = [self.body(0, head["body_frame"], "head_camera_body"),
                          self.body(2, wrist["body_frame"], "wrist_camera_body")]
        if self.publish_head_optical_axis:
            output.markers.append(self.optical_axis(1, head["optical_frame"], "head_optical_plus_z"))
        else:
            # Clear the previously published head optical arrow without
            # changing the head camera TF or its dark body marker.
            removed = Marker(); removed.header.frame_id = head["optical_frame"]
            removed.ns = "head_optical_plus_z"; removed.id = 1; removed.action = Marker.DELETE
            output.markers.append(removed)
        if self.publish_wrist_optical_axis:
            output.markers.append(self.optical_axis(3, wrist["optical_frame"], "wrist_optical_plus_z"))
        else:
            removed = Marker(); removed.header.frame_id = wrist["optical_frame"]
            removed.ns = "wrist_optical_plus_z"; removed.id = 3; removed.action = Marker.DELETE
            output.markers.append(removed)
        if self.publish_virtual_cube:
            output.markers.append(self.virtual_cube())
        self.publisher.publish(output)


def main():
    rclpy.init(); node = CameraVisualMarkers()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
