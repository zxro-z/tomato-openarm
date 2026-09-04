#!/usr/bin/python3
"""A0 fake motion: official HOME -> camera-centre observation -> HOME.

This node deliberately talks to MoveIt's planning, IK, FK, validity, and
execution services directly so the log records every safety gate.  It never
publishes hardware commands; the companion launch uses GenericSystem only.
"""
from __future__ import annotations

import copy
import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, OrientationConstraint, PlanningScene,
                             PositionConstraint, PositionIKRequest, RobotState)
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_HOME = [0.349065850399, 0.0, 0.0, 1.047197551197, 0.0, 0.0, -0.767944870878]
RIGHT_HOME = [-0.349065850399, 0.0, 0.0, 1.047197551197,
              0.0, 0.0, 0.767944870878]
SUCCESS = 1


def multiply_quaternions(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(vector, quaternion):
    x, y, z, w = quaternion
    # q * (v, 0) * q^-1, evaluated as v + w*t + qvec x t.
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def axis_angle(axis, angle):
    scale = math.sin(angle / 2.0); return (axis[0]*scale, axis[1]*scale, axis[2]*scale, math.cos(angle/2.0))


def grasp_attachment_scene(grasp: dict, tcp_to_cube_xyz=None, tcp_to_cube_quat_xyzw=None) -> PlanningScene:
    """Return the A1 PlanningScene diff: remove world cube and attach at TCP.

    The attached primitive is expressed directly in ``attach_link`` using the
    calibrated T_tcp_cube. Callers publish this diff only after a verified
    physical/fake grasp; this A0 module does not invoke it automatically.
    """
    primitive = SolidPrimitive(type=SolidPrimitive.BOX,
                               dimensions=[grasp['cube_size_m']] * 3)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = tcp_to_cube_xyz or grasp['tcp_to_cube_xyz']
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = tcp_to_cube_quat_xyzw or grasp['tcp_to_cube_quat_xyzw']
    attached = AttachedCollisionObject()
    attached.link_name = grasp['attach_link']
    attached.touch_links = grasp['touch_links']
    attached.object.id = grasp['collision_object_id']
    attached.object.header.frame_id = grasp['attach_link']
    attached.object.operation = CollisionObject.ADD
    attached.object.primitives = [primitive]
    attached.object.primitive_poses = [pose]
    scene = PlanningScene()
    scene.is_diff = True
    # MoveIt transfers an equal-ID world object into the attached-object set
    # when processing this ADD.  Do not send a separate REMOVE in the same
    # diff: some PlanningScene implementations reject that ordering.
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    return scene


class A0SingleView(Node):
    def __init__(self) -> None:
        super().__init__("baseline_a_single_view")
        self.declare_parameter("config_file", "")
        # Optional launch-time overrides; a negative value keeps the YAML value.
        self.declare_parameter("observation_distance_m", -1.0)
        self.declare_parameter("observation_hold_sec", -1.0)
        self.declare_parameter("diagnostic_distance_m", -1.0)
        path = Path(self.get_parameter("config_file").value)
        if not path.is_file():
            raise FileNotFoundError(f"baseline A config not found: {path}")
        with path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.a0, self.camera = self.config["baseline_a"], self.config["head_camera"]
        for name in ("observation_distance_m", "observation_hold_sec"):
            override = self.get_parameter(name).value
            if override >= 0.0:
                self.a0[name] = override
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        self.marker = self.create_publisher(Marker, "baseline_a/observation_target", 1)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.validity_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.execute_action = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def _joint_state(self, message: JointState) -> None:
        self.current_joints.update(zip(message.name, message.position))

    def make_grasp_attachment_scene(self, tcp_to_cube_xyz=None, tcp_to_cube_quat_xyzw=None) -> PlanningScene:
        """A1 hook; publish only after cube grasp and close verification."""
        return grasp_attachment_scene(self.a0['grasp'], tcp_to_cube_xyz, tcp_to_cube_quat_xyzw)

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f"Service failed: {client.srv_name}")
        return future.result()

    def wait_ready(self) -> None:
        for client in (self.ik_cli, self.fk_cli, self.validity_cli, self.plan_cli):
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"Service unavailable: {client.srv_name}")
        if not self.execute_action.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("Action unavailable: /execute_trajectory")
        deadline = time.monotonic() + 10.0
        while any(name not in self.current_joints for name in LEFT_JOINTS + RIGHT_JOINTS):
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for complete fake /joint_states")
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_state(self, values: dict[str, float] | None = None) -> RobotState:
        positions = values or self.current_joints
        state = RobotState()
        state.joint_state.name = LEFT_JOINTS + RIGHT_JOINTS
        state.joint_state.position = [positions[name] for name in state.joint_state.name]
        return state

    def with_official_right_home(self, state: RobotState) -> RobotState:
        """Normalize every IK/validity request to the Baseline-A right HOME."""
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        values.update(zip(RIGHT_JOINTS, RIGHT_HOME))
        return self.robot_state(values)

    def validate_bimanual_home(self) -> RobotState:
        # Allow the controller state publisher to report the completed action.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        errors = []
        expected = LEFT_HOME + RIGHT_HOME
        for name, value in zip(LEFT_JOINTS + RIGHT_JOINTS, expected):
            actual = self.current_joints[name]; error = abs(actual - value); errors.append(error)
            self.get_logger().info(f"HOME joint {name}: actual={actual:.9f} expected={value:.9f} abs_error={error:.9f}")
        if any(error > 1e-3 for error in errors):
            raise RuntimeError("HOME_START_VALIDATION_FAILED")
        self.get_logger().info("BIMANUAL_HOME_VALIDATED: all 14 joints within 1e-3 rad")
        return self.robot_state()

    def assert_right_home_unchanged(self, label: str) -> None:
        for name, expected in zip(RIGHT_JOINTS, RIGHT_HOME):
            error = abs(self.current_joints[name] - expected)
            if error > 1e-3:
                raise RuntimeError(f"RIGHT_ARM_IMMOBILITY_FAILED ({label}): {name} error={error:.9f}")
        self.get_logger().info(f"Right arm immobility ({label}): PASS")

    def valid(self, state: RobotState, label: str) -> bool:
        request = GetStateValidity.Request()
        request.robot_state, request.group_name = state, "bimanual_arms"
        response = self.call(self.validity_cli, request)
        self.get_logger().info(f"State validity ({label}): {response.valid}")
        return response.valid

    def camera_target(self, distance: float, roll_deg: float, deviation_deg: float = 0.0, azimuth_deg: float = 0.0) -> PoseStamped:
        planning_frame, optical = self.a0["planning_frame"], self.camera["optical_frame"]
        transform = self.tf_buffer.lookup_transform(planning_frame, optical, rclpy.time.Time(), timeout=Duration(seconds=5.0))
        t, q = transform.transform.translation, transform.transform.rotation
        camera_q = (q.x, q.y, q.z, q.w)
        forward = rotate((0.0, 0.0, distance), camera_q)
        # q_camera_tcp = R_tilt(deviation, azimuth) Rz(roll) Rx(pi).
        # At zero deviation TCP +Z equals camera optical -Z. R_tilt creates
        # the relaxed viewing cone without changing target xyz.
        roll = math.radians(roll_deg)
        q_roll = (0.0, 0.0, math.sin(roll / 2.0), math.cos(roll / 2.0))
        q_face = (1.0, 0.0, 0.0, 0.0)
        azimuth, deviation = math.radians(azimuth_deg), math.radians(deviation_deg)
        q_tilt = axis_angle((math.cos(azimuth), math.sin(azimuth), 0.0), deviation)
        tcp_q = multiply_quaternions(camera_q, multiply_quaternions(q_tilt, multiply_quaternions(q_roll, q_face)))
        target = PoseStamped()
        target.header.frame_id = planning_frame
        target.pose.position.x, target.pose.position.y, target.pose.position.z = t.x + forward[0], t.y + forward[1], t.z + forward[2]
        target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = tcp_q
        self.get_logger().info(
            "Camera optical frame pose (%s): xyz=(%.6f, %.6f, %.6f), q=(%.6f, %.6f, %.6f, %.6f)" %
            (optical, t.x, t.y, t.z, q.x, q.y, q.z, q.w))
        self.get_logger().info(f"observation_distance_m: {distance:.3f}; roll={roll_deg:.1f}; deviation={deviation_deg:.1f}; azimuth={azimuth_deg:.1f}")
        self.get_logger().info("Target TCP xyz: (%.6f, %.6f, %.6f)" % (target.pose.position.x, target.pose.position.y, target.pose.position.z))
        self.get_logger().info("Target TCP quaternion: (%.6f, %.6f, %.6f, %.6f)" % tcp_q)
        self.publish_target(target)
        return target

    def publish_target(self, pose: PoseStamped) -> None:
        marker = Marker(); marker.header = pose.header; marker.ns = "baseline_a"; marker.id = 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose = pose.pose; marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 1.0, 0.9
        self.marker.publish(marker)
        # RGB TCP axes let RViz audit the camera-facing family.  The blue TCP
        # +Z arrow must point opposite the optical +Z viewing ray.
        q = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        for marker_id, axis, color in ((1, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                                       (2, (0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
                                       (3, (0.0, 0.0, 1.0), (0.0, 0.3, 1.0))):
            vector = rotate(tuple(0.08 * value for value in axis), q)
            arrow = Marker(); arrow.header = pose.header; arrow.ns = "baseline_a_tcp_axes"; arrow.id = marker_id
            arrow.type, arrow.action = Marker.ARROW, Marker.ADD
            arrow.points = [Point(x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z),
                            Point(x=pose.pose.position.x + vector[0], y=pose.pose.position.y + vector[1], z=pose.pose.position.z + vector[2])]
            arrow.scale.x, arrow.scale.y = 0.006, 0.012
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = *color, 1.0
            self.marker.publish(arrow)

    def ik(self, target: PoseStamped, start: RobotState, label: str = "") -> RobotState | None:
        request = GetPositionIK.Request(); request.ik_request = PositionIKRequest()
        ik = request.ik_request; ik.group_name = self.a0["left_arm_group"]; ik.robot_state = start
        ik.avoid_collisions, ik.pose_stamped = True, target
        timeout_ns = int(self.a0["ik_timeout_sec"] * 1e9)
        ik.timeout.sec, ik.timeout.nanosec = divmod(timeout_ns, int(1e9))
        response = self.call(self.ik_cli, request)
        ok = response.error_code.val == SUCCESS
        self.get_logger().info(f"IK {label}: {ok} (code={response.error_code.val}, timeout={self.a0['ik_timeout_sec']:.2f}s)")
        return response.solution if ok else None

    def home_cost(self, solution: RobotState) -> float:
        values = dict(zip(solution.joint_state.name, solution.joint_state.position))
        return math.sqrt(sum((values[name] - home) ** 2 for name, home in zip(LEFT_JOINTS, LEFT_HOME)))

    def sweep(self, start: RobotState):
        """Dense orientation sampling: this is IK-existence evidence, not a false position-only IK claim."""
        candidates = []
        selected_distance = self.get_parameter("diagnostic_distance_m").value
        distances = [selected_distance] if selected_distance >= 0.0 else [0.20, 0.25, 0.30, 0.35]
        for distance in distances:
            reachable = []
            # Stage 1 coarse-to-fine seed: 0/10/20/30/45 cone, 90 deg
            # azimuth, 30 deg roll. Valid candidates are ranked globally;
            # no target position is altered by this search.
            for deviation in (0, 10, 20, 30, 45):
                azimuths = (0,) if deviation == 0 else (0, 90, 180, 270)
                for azimuth in azimuths:
                  for roll in range(-180, 181, 30):
                    target = self.camera_target(distance, roll, deviation, azimuth)
                    solved = None
                    for offset in self.a0["diagnostic_seed_offsets_rad"]:
                        seed_values = dict(zip(start.joint_state.name, start.joint_state.position))
                        seed_values["openarm_left_joint2"] += offset
                        seed = self.robot_state(seed_values)
                        if not self.valid(seed, f"seed d={distance:.2f} dev={deviation} az={azimuth} roll={roll:+.0f} offset={offset:+.2f}"):
                            continue
                        solution = self.ik(target, seed, f"d={distance:.2f} dev={deviation} az={azimuth} roll={roll:+.0f} seed={offset:+.2f}")
                        solution = self.with_official_right_home(solution) if solution else None
                        if solution and self.valid(solution, f"solution d={distance:.2f} dev={deviation} az={azimuth} roll={roll:+.0f}"):
                            solved = solution; break
                    if solved:
                        cost = self.home_cost(solved)
                        reachable.append((deviation, cost, roll, azimuth, target, solved))
                        self.get_logger().info(f"RELAXED d={distance:.2f} dev={deviation} az={azimuth} roll={roll:+.0f}: IK_VALID home_cost={cost:.6f}")
                    else:
                        self.get_logger().info(f"RELAXED d={distance:.2f} dev={deviation} az={azimuth} roll={roll:+.0f}: NO_IK_OR_INVALID")
            if reachable:
                best = min(reachable, key=lambda item: (item[0], item[1]))
                self.get_logger().info(f"DISTANCE SUMMARY d={distance:.2f}: valid={len(reachable)} min_error={best[0]:.1f} best_roll={best[2]:+.0f} best_cost={best[1]:.6f}")
                candidates.extend((distance, *item) for item in reachable)
            else:
                self.get_logger().info(f"DISTANCE SUMMARY d={distance:.2f}: reachable=0")
        return candidates

    @staticmethod
    def joint_goal(names, values):
        return Constraints(joint_constraints=[JointConstraint(joint_name=name, position=value,
            tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0) for name, value in zip(names, values)])

    def pose_goal(self, target: PoseStamped) -> Constraints:
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[self.a0["position_tolerance_m"] * 2] * 3)
        position = PositionConstraint(); position.header = target.header; position.link_name = self.a0["left_tcp_frame"]
        position.constraint_region.primitives = [box]; position.constraint_region.primitive_poses = [copy.deepcopy(target.pose)]; position.weight = 1.0
        orientation = OrientationConstraint(); orientation.header = target.header; orientation.link_name = self.a0["left_tcp_frame"]
        orientation.orientation = target.pose.orientation; orientation.absolute_x_axis_tolerance = self.a0["orientation_tolerance_rad"]
        orientation.absolute_y_axis_tolerance = self.a0["orientation_tolerance_rad"]; orientation.absolute_z_axis_tolerance = self.a0["orientation_tolerance_rad"]; orientation.weight = 1.0
        return Constraints(position_constraints=[position], orientation_constraints=[orientation])

    def plan(self, start: RobotState, goal: Constraints, label: str, group: str | None = None):
        request = GetMotionPlan.Request(); motion = request.motion_plan_request
        motion.group_name, motion.start_state = group or self.a0["left_arm_group"], start
        motion.goal_constraints = [goal]; motion.num_planning_attempts, motion.allowed_planning_time = 3, 5.0
        motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = 0.2
        response = self.call(self.plan_cli, request).motion_plan_response
        ok = response.error_code.val == SUCCESS and bool(response.trajectory.joint_trajectory.points)
        points = len(response.trajectory.joint_trajectory.points)
        self.get_logger().info(f"Planning success ({label}): {ok}; trajectory points: {points}; code={response.error_code.val}")
        if not ok: raise RuntimeError(f"Planning failed for {label}")
        self.validate_trajectory(response.trajectory, start, label)
        return response.trajectory

    def validate_trajectory(self, trajectory, start: RobotState, label: str) -> None:
        names = trajectory.joint_trajectory.joint_names
        start_values = dict(zip(start.joint_state.name, start.joint_state.position))
        for index, point in enumerate(trajectory.joint_trajectory.points):
            values = dict(start_values); values.update(zip(names, point.positions))
            if not self.valid(self.robot_state(values), f"{label} point {index}"):
                raise RuntimeError(f"Self-collision / invalid state at {label} trajectory point {index}")

    def execute(self, trajectory, label: str) -> None:
        goal = ExecuteTrajectory.Goal(); goal.trajectory = trajectory
        future = self.execute_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Execution goal rejected for {label}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result().result
        ok = response.error_code.val == SUCCESS
        self.get_logger().info(f"Fake execution success ({label}): {ok} (code={response.error_code.val})")
        if not ok: raise RuntimeError(f"Execution failed for {label}")

    def fk_error(self, state: RobotState, target: PoseStamped) -> float:
        request = GetPositionFK.Request(); request.header.frame_id = self.a0["planning_frame"]
        request.fk_link_names = [self.a0["left_tcp_frame"]]; request.robot_state = state
        response = self.call(self.fk_cli, request)
        if response.error_code.val != SUCCESS or not response.pose_stamped: raise RuntimeError("Final TCP FK failed")
        pose = response.pose_stamped[0].pose; dx = pose.position.x - target.pose.position.x; dy = pose.position.y - target.pose.position.y; dz = pose.position.z - target.pose.position.z
        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        self.get_logger().info("Final TCP FK xyz: (%.6f, %.6f, %.6f), q=(%.6f, %.6f, %.6f, %.6f); final TCP error: %.6f m" %
            (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, error))
        return error

    def run(self) -> None:
        self.wait_ready(); start = self.robot_state()
        if not self.valid(start, "current start"): raise RuntimeError("Current state is invalid")
        # Baseline-A contract: initialize both arms with SRDF bimanual_arms/home.
        home = self.plan(start, self.joint_goal(LEFT_JOINTS + RIGHT_JOINTS, LEFT_HOME + RIGHT_HOME),
                         "bimanual HOME", group="bimanual_arms")
        self.execute(home, "bimanual HOME")
        start = self.validate_bimanual_home()
        self.assert_right_home_unchanged("before sweep")
        candidates = self.sweep(start)
        if not candidates:
            raise RuntimeError("RELAXED_CAMERA_FACING_UNREACHABLE: no collision-valid candidate in the sampled cone")
        # Rank camera-facing error first, then HOME joint-space cost.
        distance, deviation, cost, roll, azimuth, target, solution = min(candidates, key=lambda item: (item[1], item[2]))
        self.get_logger().info(f"BEST candidate: distance={distance:.2f} deviation={deviation:.1f} azimuth={azimuth:.1f} roll={roll:+.0f} home_cost={cost:.6f}")
        self.get_logger().info(f"BEST joints: {dict(zip(solution.joint_state.name, solution.joint_state.position))}")
        if deviation > 20.0:
            raise RuntimeError(f"NO_ACCEPTABLE_VIEWING_POSE: best camera_facing_error={deviation:.1f} deg; execution gate is <=20 deg")
        trajectory = self.plan(start, self.pose_goal(target), "observation")
        self.execute(trajectory, "observation")
        self.assert_right_home_unchanged("after observation")
        final_values = dict(zip(start.joint_state.name, start.joint_state.position)); final_values.update(zip(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[-1].positions))
        error = self.fk_error(self.robot_state(final_values), target)
        if error > self.a0["final_position_tolerance_m"]: raise RuntimeError(f"Final TCP error exceeds tolerance: {error:.6f} m")
        self.get_logger().info(f"Observation hold: {self.a0['observation_hold_sec']:.2f} sec")
        time.sleep(self.a0["observation_hold_sec"])
        return_home = self.plan(self.robot_state(final_values), self.joint_goal(LEFT_JOINTS, LEFT_HOME), "return HOME")
        self.execute(return_home, "return HOME")
        self.assert_right_home_unchanged("after return HOME")
        self.get_logger().info("A0 PASS: HOME -> observation pose -> HOME")


def main() -> None:
    rclpy.init(); node = A0SingleView()
    try: node.run()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
