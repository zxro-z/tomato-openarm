#!/usr/bin/python3
"""Move an attached cube's configured top face to the head camera centre.

No pick, detach, gripper command, image capture, or marker detection is done
here.  The only target is derived as top face -> cube -> TCP.
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionFK
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from baseline_a_single_view import A0SingleView, LEFT_HOME, LEFT_JOINTS, rotate


def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qinv(q):
    x, y, z, w = q
    return -x, -y, -z, w


def q_axis_angle(axis, angle):
    scale = math.sin(angle / 2.0)
    return axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(angle / 2.0)


def q_align(source, target):
    """Shortest quaternion mapping a unit source vector onto target."""
    dot = sum(source[i] * target[i] for i in range(3))
    if dot < -0.999999:
        # Pick a stable axis perpendicular to source for the 180-degree case.
        axis = (0., 1., 0.) if abs(source[1]) < .9 else (1., 0., 0.)
        return q_axis_angle(axis, math.pi)
    cross = (source[1] * target[2] - source[2] * target[1],
             source[2] * target[0] - source[0] * target[2],
             source[0] * target[1] - source[1] * target[0])
    q = (*cross, 1. + dot)
    norm = math.sqrt(sum(value * value for value in q))
    return tuple(value / norm for value in q)


def compose(p_a, q_a, p_b, q_b):
    offset = rotate(p_b, q_a)
    return tuple(p_a[i] + offset[i] for i in range(3)), qmul(q_a, q_b)


def inverse(p, q):
    q_i = qinv(q)
    return tuple(-value for value in rotate(p, q_i)), q_i


class MoveToCamera(A0SingleView):
    def __init__(self):
        super().__init__()
        self.observation = self.a0['observation']
        self.grasp = self.a0['grasp']
        self.declare_parameter('return_home', False)
        self.grasp_ready = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'baseline_a/grasp_calibration_ready', self._ready, qos)
        self.scene_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.debug_pub = self.create_publisher(MarkerArray, 'baseline_a/observation_debug', 1)

    def _ready(self, message):
        self.grasp_ready = message.data

    def robot_state(self, values=None):
        """Preserve the verified closed fingers in all IK/validity requests."""
        state = super().robot_state(values)
        value_map = values or self.current_joints
        for name in self.grasp['calibration_open_joint_names']:
            state.joint_state.name.append(name)
            state.joint_state.position.append(value_map.get(name, self.grasp['grasp_close_joint_m']))
        return state

    def wait_for_grasp(self):
        deadline = time.monotonic() + 35.0
        while rclpy.ok() and not self.grasp_ready:
            if time.monotonic() > deadline:
                raise RuntimeError('ERROR: grasp-ready signal timed out')
            rclpy.spin_once(self, timeout_sec=.1)
        if not self.scene_cli.wait_for_service(timeout_sec=15.0):
            raise RuntimeError('GetPlanningScene unavailable')
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                         PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        scene = self.call(self.scene_cli, request).scene
        attached = [item for item in scene.robot_state.attached_collision_objects
                    if item.object.id == self.grasp['collision_object_id']]
        if len(attached) != 1:
            raise RuntimeError('ERROR: expected attached cube not found')
        cube = attached[0]
        if cube.link_name != self.grasp['attach_link']:
            raise RuntimeError(f'ERROR: attached link={cube.link_name}, expected={self.grasp["attach_link"]}')
        if sorted(cube.touch_links) != sorted(self.grasp['touch_links']):
            raise RuntimeError(f'ERROR: attached touch_links mismatch={cube.touch_links}')
        if any(item.id == self.grasp['collision_object_id'] for item in scene.world.collision_objects):
            raise RuntimeError('ERROR: attached cube has a world duplicate')
        if not any(item.id == 'table' for item in scene.world.collision_objects):
            raise RuntimeError('ERROR: table collision object not found; refusing to plan without table safety')
        for name in self.grasp['calibration_open_joint_names']:
            actual = self.current_joints[name]
            if abs(actual - self.grasp['grasp_close_joint_m']) > self.grasp['calibration_open_tolerance_m']:
                raise RuntimeError(f'ERROR: closed gripper required; {name}={actual:.9f}')
        self.get_logger().info(f'ATTACHED_CUBE_VERIFIED: {cube.object.id} on {cube.link_name}; touch_links={cube.touch_links}')
        self.get_logger().info('TABLE_COLLISION_VERIFIED: table')

    def camera_pose(self):
        transform = self.tf_buffer.lookup_transform(self.a0['planning_frame'], self.observation['camera_frame'], rclpy.time.Time())
        t, q = transform.transform.translation, transform.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def target_for_roll(self, roll_deg):
        """T_world_tcp = T_world_camera T_camera_top inv(T_cube_top) inv(T_tcp_cube)."""
        camera_p, camera_q = self.camera_pose()
        half = self.grasp['cube_size_m'] / 2.0
        axis = tuple(self.observation['top_face_axis'])
        # Rz(roll) R_align: the configured cube-local top normal maps to
        # camera optical -Z; Rz preserves that normal and varies only roll.
        camera_cube_q = qmul(q_axis_angle((0., 0., 1.), math.radians(roll_deg)),
                             q_align(axis, (0., 0., -1.)))
        # T_camera_top: p=(0,0,z_opt); T_cube_top: translation +half*axis.
        camera_cube_p, _ = compose((0., 0., self.observation['optimal_distance_m']), camera_cube_q,
                                   tuple(-half * value for value in axis), (0., 0., 0., 1.))
        cube_tcp_p, cube_tcp_q = inverse(tuple(self.grasp['tcp_to_cube_xyz']),
                                          tuple(self.grasp['tcp_to_cube_quat_xyzw']))
        camera_tcp_p, camera_tcp_q = compose(camera_cube_p, camera_cube_q, cube_tcp_p, cube_tcp_q)
        world_tcp_p, world_tcp_q = compose(camera_p, camera_q, camera_tcp_p, camera_tcp_q)
        target = PoseStamped(); target.header.frame_id = self.a0['planning_frame']
        target.pose.position.x, target.pose.position.y, target.pose.position.z = world_tcp_p
        target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = world_tcp_q
        return target

    def current_cost(self, solution, start):
        candidate = dict(zip(solution.joint_state.name, solution.joint_state.position))
        current = dict(zip(start.joint_state.name, start.joint_state.position))
        return math.sqrt(sum((candidate[name] - current[name]) ** 2 for name in LEFT_JOINTS))

    def marker_arrow(self, marker_id, start, vector, color):
        marker = Marker(); marker.header.frame_id = self.a0['planning_frame']; marker.ns = 'baseline_a_observation_debug'; marker.id = marker_id
        marker.type = Marker.ARROW; marker.action = Marker.ADD
        marker.points = [Point(x=start[0], y=start[1], z=start[2]), Point(x=start[0] + vector[0], y=start[1] + vector[1], z=start[2] + vector[2])]
        marker.scale.x, marker.scale.y = .006, .012; marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.
        return marker

    def publish_debug(self, tcp_pose, actual_top=None):
        tcp_p = (tcp_pose.pose.position.x, tcp_pose.pose.position.y, tcp_pose.pose.position.z)
        tcp_q = (tcp_pose.pose.orientation.x, tcp_pose.pose.orientation.y, tcp_pose.pose.orientation.z, tcp_pose.pose.orientation.w)
        cube_p, cube_q = compose(tcp_p, tcp_q, tuple(self.grasp['tcp_to_cube_xyz']), tuple(self.grasp['tcp_to_cube_quat_xyzw']))
        axis = tuple(self.observation['top_face_axis'])
        top_offset = rotate(tuple(self.grasp['cube_size_m'] * .5 * value for value in axis), cube_q)
        top = tuple(cube_p[i] + top_offset[i] for i in range(3))
        output = MarkerArray()
        # ID 0 was the cyan camera-forward arrow.  The camera TF remains the
        # calculation source, but this Baseline-A view intentionally omits its
        # visual arrow to avoid obscuring the observation scene.  DELETE also
        # clears an arrow retained by an already-open RViz instance.
        removed = Marker(); removed.header.frame_id = self.a0['planning_frame']; removed.ns = 'baseline_a_observation_debug'; removed.id = 0; removed.action = Marker.DELETE
        output.markers.append(removed)
        sphere = Marker(); sphere.header.frame_id = self.a0['planning_frame']; sphere.ns = 'baseline_a_observation_debug'; sphere.id = 1; sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = top; sphere.pose.orientation.w = 1.; sphere.scale.x = sphere.scale.y = sphere.scale.z = .018; sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = .1, 1., .3, 1.; output.markers.append(sphere)
        output.markers.append(self.marker_arrow(2, top, rotate(tuple(.08 * value for value in axis), cube_q), (1., .5, .1)))
        if actual_top:
            actual = Marker(); actual.header.frame_id = self.a0['planning_frame']; actual.ns = 'baseline_a_observation_debug'; actual.id = 3; actual.type = Marker.SPHERE; actual.action = Marker.ADD
            actual.pose.position.x, actual.pose.position.y, actual.pose.position.z = actual_top; actual.pose.orientation.w = 1.; actual.scale.x = actual.scale.y = actual.scale.z = .014; actual.color.r, actual.color.g, actual.color.b, actual.color.a = 1., .1, .8, 1.; output.markers.append(actual)
        self.debug_pub.publish(output)

    def final_geometry(self, final_state):
        request = GetPositionFK.Request(); request.header.frame_id = self.a0['planning_frame']; request.fk_link_names = [self.a0['left_tcp_frame']]; request.robot_state = final_state
        response = self.call(self.fk_cli, request)
        if not response.pose_stamped:
            raise RuntimeError('ERROR: final TCP FK failed')
        pose = response.pose_stamped[0].pose; tcp_p = (pose.position.x, pose.position.y, pose.position.z); tcp_q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        cube_p, cube_q = compose(tcp_p, tcp_q, tuple(self.grasp['tcp_to_cube_xyz']), tuple(self.grasp['tcp_to_cube_quat_xyzw']))
        axis = tuple(self.observation['top_face_axis']); top = tuple(cube_p[i] + rotate(tuple(self.grasp['cube_size_m'] / 2.0 * value for value in axis), cube_q)[i] for i in range(3))
        camera_p, camera_q = self.camera_pose(); camera_world_p, camera_world_q = inverse(camera_p, camera_q)
        top_camera, _ = compose(camera_world_p, camera_world_q, top, (0., 0., 0., 1.))
        lateral = math.hypot(top_camera[0], top_camera[1]); distance_error = top_camera[2] - self.observation['optimal_distance_m']
        top_normal = rotate(axis, cube_q); camera_to_object = tuple(-value for value in rotate((0., 0., 1.), camera_q))
        angle = math.degrees(math.acos(max(-1., min(1., sum(top_normal[i] * camera_to_object[i] for i in range(3))))))
        self.get_logger().info('FINAL_TOP_GEOMETRY: lateral_error_m=%.6f camera_to_top_m=%.6f distance_error_m=%.6f top_normal_error_deg=%.6f' % (lateral, top_camera[2], distance_error, angle))
        return top

    def run(self):
        self.wait_ready(); self.wait_for_grasp()
        start = self.robot_state()
        if not self.valid(start, 'attached-cube move start'):
            raise RuntimeError('ERROR: attached-cube start state invalid')
        self.assert_right_home_unchanged('before camera move')
        candidates = []
        for roll in self.observation['roll_candidates_deg']:
            target = self.target_for_roll(roll)
            solution = self.ik(target, start, f'top roll={roll:+.0f}')
            if solution:
                solution = self.with_official_right_home(solution)
                if self.valid(solution, f'top roll={roll:+.0f}'):
                    candidates.append((self.current_cost(solution, start), abs(roll), roll, target))
        if not candidates:
            raise RuntimeError('NO_VALID_TOP_FACE_CAMERA_POSE')
        trajectory = None
        for cost, _, roll, target in sorted(candidates):
            try:
                self.publish_debug(target)
                trajectory = self.plan(start, self.pose_goal(target), f'top-face camera move roll={roll:+.0f}')
                chosen = cost, roll, target
                break
            except RuntimeError as error:
                self.get_logger().warning(f'roll={roll:+.0f} plan failed: {error}')
        if trajectory is None:
            raise RuntimeError('NO_VALID_TOP_FACE_CAMERA_PLAN')
        cost, roll, target = chosen
        self.get_logger().info('SELECTED_TOP_FACE_POSE: roll_deg=%+.0f current_joint_cost=%.6f TCP xyz=(%.6f, %.6f, %.6f) q=(%.6f, %.6f, %.6f, %.6f)' % (roll, cost, target.pose.position.x, target.pose.position.y, target.pose.position.z, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w))
        self.execute(trajectory, 'top-face camera move')
        final_state = self.robot_state()
        if not self.valid(final_state, 'top-face observation final'):
            raise RuntimeError('ERROR: final observation state invalid')
        self.assert_right_home_unchanged('after camera move')
        top = self.final_geometry(final_state); self.publish_debug(target, top)
        self.get_logger().info(f'BASELINE_A_OBSERVATION_POSE_READY: hold_sec={self.observation["hold_sec"]:.2f}')
        time.sleep(self.observation['hold_sec'])
        if self.get_parameter('return_home').value:
            home = self.plan(final_state, self.joint_goal(LEFT_JOINTS, LEFT_HOME), 'optional return HOME')
            self.execute(home, 'optional return HOME')
            self.assert_right_home_unchanged('after optional return HOME')


def main():
    rclpy.init(); node = MoveToCamera()
    try:
        node.run(); rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
