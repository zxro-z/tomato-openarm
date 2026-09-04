#!/usr/bin/python3
import argparse
import sys
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from shape_msgs.msg import SolidPrimitive
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


TABLE_COLLISION_SIZE = (1.6, 1.0, 0.04)
TABLE_COLLISION_POS = (0.0, 0.0, -0.0211)
TABLE_COLLISION_ROT_XYZW = (0.0, 0.0, 0.0, 1.0)

TOP_CAMERA_POS = (-0.38725, 0.09957, 0.78544)
TOP_CAMERA_ROT_WXYZ = (0.7071067811865476, 0.0, 0.7071067811865475, 0.0)
WRIST_CAMERA_POS = (0.06336, -0.01314, -0.05143)
WRIST_CAMERA_ROT_WXYZ = (0.0, 0.5, 0.0, 0.86603)


def wxyz_to_xyzw(quat_wxyz: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])


def make_pose(
    position: tuple[float, float, float],
    rotation_xyzw: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = rotation_xyzw[0]
    pose.orientation.y = rotation_xyzw[1]
    pose.orientation.z = rotation_xyzw[2]
    pose.orientation.w = rotation_xyzw[3]
    return pose


def make_box(
    object_id: str,
    frame_id: str,
    position: tuple[float, float, float],
    rotation_xyzw: tuple[float, float, float, float],
    size: tuple[float, float, float],
) -> CollisionObject:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)

    collision = CollisionObject()
    collision.id = object_id
    collision.header.frame_id = frame_id
    collision.operation = CollisionObject.ADD
    collision.primitives = [primitive]
    collision.primitive_poses = [make_pose(position, rotation_xyzw)]
    return collision


def make_static_tf(
    parent: str,
    child: str,
    position: tuple[float, float, float],
    rotation_xyzw: tuple[float, float, float, float],
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = position[0]
    transform.transform.translation.y = position[1]
    transform.transform.translation.z = position[2]
    transform.transform.rotation.x = rotation_xyzw[0]
    transform.transform.rotation.y = rotation_xyzw[1]
    transform.transform.rotation.z = rotation_xyzw[2]
    transform.transform.rotation.w = rotation_xyzw[3]
    return transform


class CubePnPSceneNode(Node):
    def __init__(self, publish_period: float):
        super().__init__("cube_pnp_scene")
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.scene_msg = PlanningScene()
        self.scene_msg.is_diff = True
        self.scene_msg.world.collision_objects = [
            make_box("table", "world", TABLE_COLLISION_POS, TABLE_COLLISION_ROT_XYZW, TABLE_COLLISION_SIZE),
        ]
        self.static_tfs = [
            make_static_tf("world", "top_camera_link", TOP_CAMERA_POS, wxyz_to_xyzw(TOP_CAMERA_ROT_WXYZ)),
            make_static_tf(
                "openarm_left_hand_tcp",
                "wrist_camera_link",
                WRIST_CAMERA_POS,
                wxyz_to_xyzw(WRIST_CAMERA_ROT_WXYZ),
            ),
        ]
        self.tf_broadcaster.sendTransform(self.static_tfs)
        self.timer = self.create_timer(publish_period, self.publish_scene)
        self.publish_scene()

    def publish_scene(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for obj in self.scene_msg.world.collision_objects:
            obj.header.stamp = stamp
        for transform in self.static_tfs:
            transform.header.stamp = stamp
        self.tf_broadcaster.sendTransform(self.static_tfs)
        self.scene_pub.publish(self.scene_msg)


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish the LeRobot table planning scene and camera TFs.")
    parser.add_argument("--publish-period", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> None:
    ros_args = sys.argv if argv is None else list(argv)
    rclpy.init(args=ros_args)
    cli_args = remove_ros_args(args=ros_args)
    parsed_args = parse_args(cli_args[1:])
    node = CubePnPSceneNode(parsed_args.publish_period)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
