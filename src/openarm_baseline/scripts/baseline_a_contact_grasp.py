#!/usr/bin/python3
"""Geometry-only fake grasp: basic-finger audit -> close -> attach -> hold.

This does not model gravity, friction, or gripping force.  It uses URDF mesh
geometry and MoveIt collision/attachment semantics only.
"""
from __future__ import annotations

import math
import struct
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from baseline_a_single_view import A0SingleView, LEFT_HOME, rotate


def mesh_vertices(relative_path):
    """Read the active basic-finger STL without relying on a tactile mesh.

    OpenArm's collision finger mesh is binary STL.  Keeping the mesh vertices
    lets the grasp reference be derived from the same geometry MoveIt uses.
    """
    path = Path(get_package_share_directory('openarm_description')) / relative_path
    data = path.read_bytes()
    if len(data) < 84:
        raise RuntimeError(f'Invalid STL collision mesh: {path}')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) == 84 + count * 50:
        vertices = []
        for index in range(count):
            vertices.extend(struct.unpack_from('<9f', data, 84 + index * 50 + 12)[offset:offset + 3]
                            for offset in (0, 3, 6))
        return [tuple(vertex) for vertex in vertices]
    # Retain an ASCII-STL fallback for a future collision-mesh substitution.
    vertices = [tuple(map(float, line.split()[1:4])) for line in data.decode('utf-8').splitlines()
                if line.lstrip().startswith('vertex ')]
    if not vertices:
        raise RuntimeError(f'No vertices in collision mesh: {path}')
    return vertices


class ContactGrasp(A0SingleView):
    def __init__(self):
        super().__init__()
        self.grasp = self.a0['grasp']; self.contact = self.grasp['contact_geometry']
        self.open_ready = False
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'baseline_a/grasp_calibration_open_ready', self._open_ready, ready_qos)
        self.ready_pub = self.create_publisher(Bool, 'baseline_a/grasp_calibration_ready', ready_qos)
        self.pose_pub = self.create_publisher(PoseStamped, 'baseline_a/contact_grasp_tcp_pose', 1)
        self.marker_pub = self.create_publisher(MarkerArray, 'baseline_a/grasp_contact_geometry', 1)
        self.apply_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/left_gripper_controller/follow_joint_trajectory')

    def _open_ready(self, message): self.open_ready = message.data

    def collision_vertices_in_tcp(self, side):
        link = self.contact[f'{side}_contact_link']
        mesh = self.contact[f'{side}_collision_mesh']
        origin = self.contact[f'{side}_collision_origin_xyz']
        scale = self.contact[f'{side}_collision_scale_xyz']
        transform = self.tf_buffer.lookup_transform(self.a0['left_tcp_frame'], link, rclpy.time.Time()).transform
        q = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        return [tuple(translation[index] + rotate(
                     tuple(origin[axis] + scale[axis] * vertex[axis] for axis in range(3)), q)[index]
                     for index in range(3)) for vertex in mesh_vertices(mesh)]

    def contacts(self):
        left_vertices = self.collision_vertices_in_tcp('left')
        right_vertices = self.collision_vertices_in_tcp('right')
        # In the TCP frame the left finger's inner collision face is +Y-facing
        # (minimum Y); the mirrored right finger's is -Y-facing (maximum Y).
        left_face_y = min(point[1] for point in left_vertices)
        right_face_y = max(point[1] for point in right_vertices)
        x_center = (min(point[0] for point in left_vertices) + max(point[0] for point in left_vertices) +
                    min(point[0] for point in right_vertices) + max(point[0] for point in right_vertices)) / 4.0
        # TCP +Z is forward.  The cube stays entirely within the finger's
        # forward extent: center_z + half_cube = common collision-mesh tip.
        tip_z = min(max(point[2] for point in vertices) for vertices in (left_vertices, right_vertices))
        center = (x_center, (left_face_y + right_face_y) / 2.0,
                  tip_z - self.grasp['cube_size_m'] / 2.0)
        left = (center[0], left_face_y, center[2])
        right = (center[0], right_face_y, center[2])
        return left, right, center

    def geometry_markers(self, left, right, center, gap):
        result = MarkerArray(); markers=[]
        for ident, point, color in ((0,left,(1.,.2,1.)), (1,right,(1.,.2,1.)),
                                    (2,(center[0], center[1]+.025, center[2]),(.2,1.,.2)),
                                    (3,(center[0], center[1]-.025, center[2]),(.2,1.,.2))):
            marker=Marker(); marker.header.frame_id=self.a0['left_tcp_frame']; marker.ns='grasp_contact'; marker.id=ident; marker.type=Marker.SPHERE; marker.action=Marker.ADD
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=point; marker.pose.orientation.w=1.; marker.scale.x=marker.scale.y=marker.scale.z=.012
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=*color,1.; markers.append(marker)
        text=Marker(); text.header.frame_id=self.a0['left_tcp_frame']; text.ns='grasp_contact'; text.id=4; text.type=Marker.TEXT_VIEW_FACING; text.action=Marker.ADD
        text.pose.position.x,text.pose.position.y,text.pose.position.z=center[0],center[1],center[2]+.035; text.pose.orientation.w=1.; text.scale.z=.018; text.color.r=text.color.g=text.color.b=text.color.a=1.
        text.text=f'finger inner gap={gap*1000:.1f} mm; cube gap={(gap-self.grasp["cube_size_m"])*1000:.1f} mm'; markers.append(text); result.markers=markers; self.marker_pub.publish(result)

    def apply(self, scene, label):
        if not self.apply_cli.wait_for_service(timeout_sec=10.): raise RuntimeError('ApplyPlanningScene unavailable')
        request=ApplyPlanningScene.Request(); request.scene=scene; response=self.call(self.apply_cli, request)
        if not response.success: raise RuntimeError(f'PlanningScene apply failed: {label}')
        self.get_logger().info(f'PLANNING_SCENE_APPLY_SUCCESS: {label}')

    def world_cube_scene(self, center):
        primitive=SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[self.grasp['cube_size_m']]*3)
        cube=CollisionObject(); cube.id=self.grasp['collision_object_id']; cube.header.frame_id=self.a0['left_tcp_frame']; cube.operation=CollisionObject.ADD
        cube.primitives=[primitive]; cube.primitive_poses[0:0]=[]
        pose=PoseStamped().pose; pose.position.x,pose.position.y,pose.position.z=center; pose.orientation.w=1.; cube.primitive_poses=[pose]
        scene=PlanningScene(); scene.is_diff=True; scene.world.collision_objects=[cube]; return scene

    def send_gripper(self, value):
        if not self.gripper.wait_for_server(timeout_sec=10.): raise RuntimeError('left gripper action unavailable')
        goal=FollowJointTrajectory.Goal(); goal.trajectory.joint_names=self.grasp['calibration_open_joint_names']
        point=JointTrajectoryPoint(); point.positions=[value, value]; point.time_from_start.sec=1; goal.trajectory.points=[point]
        future=self.gripper.send_goal_async(goal); rclpy.spin_until_future_complete(self, future); handle=future.result()
        if handle is None or not handle.accepted: raise RuntimeError('close goal rejected')
        future=handle.get_result_async(); rclpy.spin_until_future_complete(self, future)
        if future.result().result.error_code != 0: raise RuntimeError('close execution failed')
        for _ in range(10): rclpy.spin_once(self, timeout_sec=.05)

    def run_grasp(self):
        self.wait_ready()
        left,right,center=self.contacts(); open_value=self.current_joints[self.grasp['calibration_open_joint_names'][0]]
        gap=abs(left[1]-right[1]); calculated_close=max(0., min(.044, open_value-(gap-self.grasp['cube_size_m'])/2))
        close=self.grasp['grasp_close_joint_m']
        base=self.grasp['tcp_to_cube_xyz']; delta=tuple(center[i]-base[i] for i in range(3))
        self.get_logger().info(f'LEFT BASIC_FINGER_INNER_FACE {self.contact["left_contact_link"]}: tcp={left}')
        self.get_logger().info(f'RIGHT BASIC_FINGER_INNER_FACE {self.contact["right_contact_link"]}: tcp={right}')
        self.get_logger().info(f'BASIC_FINGER_GRASP_CENTER tcp={center}; CONFIG T_tcp_cube={base}; delta={delta}; norm={math.sqrt(sum(v*v for v in delta)):.6f}m')
        for value in [round(open_value-i*self.contact['close_scan_step_m'], 3) for i in range(int(open_value/self.contact['close_scan_step_m'])+1)]:
            predicted=gap-2*(open_value-value); self.get_logger().info(f'FINGER_SCAN joint={value:.3f} inner_gap={predicted:.4f} cube_gap={predicted-self.grasp["cube_size_m"]:.4f}')
        self.get_logger().info(f'RECOMMENDED_GRASP_CLOSE_JOINT={calculated_close:.9f}m; configured={close:.9f}m')
        if abs(calculated_close-close) > self.contact['contact_gap_tolerance_m']:
            raise RuntimeError('SENSORLESS_GRASP_CLOSE_CONFIG_MISMATCH')
        # The config records the mesh/FK-derived reference so the attached
        # cube transform remains stable after the close command.
        verified_center=tuple(self.grasp['tcp_to_cube_xyz'])
        pose=PoseStamped(); pose.header.frame_id=self.a0['left_tcp_frame']; pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=verified_center; pose.pose.orientation.w=1.; self.pose_pub.publish(pose)
        self.geometry_markers(left,right,verified_center,gap); self.apply(self.world_cube_scene(verified_center),'world cube before close')
        self.send_gripper(close)
        for name in self.grasp['calibration_open_joint_names']:
            error=abs(self.current_joints[name]-close); self.get_logger().info(f'GRASP_CLOSE joint {name}: actual={self.current_joints[name]:.9f} expected={close:.9f} abs_error={error:.9f}')
            if error > self.grasp['calibration_open_tolerance_m']: raise RuntimeError('GRIPPER_CLOSE_VALIDATION_FAILED')
        self.get_logger().info('GRIPPER_CLOSE_VALIDATED')
        left,right,center_after=self.contacts(); closed_gap=abs(left[1]-right[1]); self.geometry_markers(left,right,verified_center,closed_gap)
        self.apply(self.make_grasp_attachment_scene(verified_center, (0.,0.,0.,1.)), 'world->attached cube')
        self.get_logger().info(f'CUBE_ATTACHED link={self.grasp["attach_link"]} touch_links={self.grasp["touch_links"]}')
        values=dict(self.current_joints); values['openarm_left_joint1'] += math.radians(5.)
        try:
            trajectory=self.plan(self.robot_state(), self.joint_goal(['openarm_left_joint1'], [values['openarm_left_joint1']]), 'attached-cube hold motion')
            self.execute(trajectory, 'attached-cube hold motion')
            back=self.plan(self.robot_state(), self.joint_goal(['openarm_left_joint1'], [LEFT_HOME[0]]), 'attached-cube smoke return')
            self.execute(back, 'attached-cube smoke return'); self.get_logger().info('ATTACHED_CUBE_HOLD_SMOKE_TEST: PASS')
        except Exception as error: self.get_logger().error(f'ATTACHED_CUBE_HOLD_SMOKE_TEST: FAIL ({error})')
        # The smoke motion is not the calibration reference.  Return both arms
        # to the official HOME without commanding either finger joint.
        final_home=self.plan(self.robot_state(), self.joint_goal(
            [f'openarm_left_joint{i}' for i in range(1,8)] + [f'openarm_right_joint{i}' for i in range(1,8)],
            LEFT_HOME + [-0.349065850399, 0.0, 0.0, 1.047197551197, 0.0, 0.0, 0.767944870878]),
            'final attached-cube bimanual HOME', group='bimanual_arms')
        before=[self.current_joints[name] for name in self.grasp['calibration_open_joint_names']]
        self.execute(final_home, 'final attached-cube bimanual HOME'); self.validate_bimanual_home()
        for name, before_value in zip(self.grasp['calibration_open_joint_names'], before):
            actual=self.current_joints[name]; error=abs(actual-close)
            self.get_logger().info(f'FINAL_GRIPPER joint {name}: before_home={before_value:.9f} after_home={actual:.9f} expected={close:.9f} abs_error={error:.9f}')
            if error > self.grasp['calibration_open_tolerance_m']: raise RuntimeError('FINAL_GRIPPER_CLOSED_VALIDATION_FAILED')
        if not self.get_scene_cli.wait_for_service(timeout_sec=10.): raise RuntimeError('GetPlanningScene unavailable')
        request=GetPlanningScene.Request(); request.components.components=12; scene=self.call(self.get_scene_cli, request).scene
        attached=[obj for obj in scene.robot_state.attached_collision_objects if obj.object.id == self.grasp['collision_object_id']]
        world=[obj for obj in scene.world.collision_objects if obj.id == self.grasp['collision_object_id']]
        if len(attached) != 1 or world: raise RuntimeError('FINAL_ATTACHED_CUBE_VALIDATION_FAILED')
        p=attached[0].object.pose.position
        if math.dist((p.x,p.y,p.z), verified_center) > 1e-6: raise RuntimeError('FINAL_T_TCP_CUBE_VALIDATION_FAILED')
        self.get_logger().info('GRASP_HOME_FINAL_VALIDATED: arms HOME, gripper closed, one attached cube, no world duplicate')
        self.ready_pub.publish(Bool(data=True)); self.get_logger().info('BASELINE_A_GRASP_HOME_READY')

def main():
    rclpy.init(); node=ContactGrasp()
    try:
        # Keep service/action waits outside subscription callbacks; otherwise a
        # single-threaded executor cannot dispatch their responses.
        while rclpy.ok() and not node.open_ready: rclpy.spin_once(node, timeout_sec=.2)
        if node.open_ready: node.run_grasp()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
