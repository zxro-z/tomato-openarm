#!/usr/bin/python3
"""Diagnostic-only A0 position reachability test; never executes a trajectory."""
from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

LEFT = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT = [f"openarm_right_joint{i}" for i in range(1, 8)]
LH = [0.349065850399, 0., 0., 1.047197551197, 0., 0., -0.767944870878]
RH = [-0.349065850399, 0., 0., 1.047197551197, 0., 0., .767944870878]


def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)


def rotate(v, q):
    x, y, z, w = q; vx, vy, vz = v
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def uniform_quaternions(count: int):
    """Deterministic Shoemake SO(3) samples using two irrational rotations."""
    phi, psi = (math.sqrt(5)-1)/2, math.sqrt(2)-1
    for i in range(count):
        u1 = (i + .5) / count; u2 = (i * phi) % 1.; u3 = (i * psi) % 1.
        yield (math.sqrt(1-u1)*math.sin(2*math.pi*u2), math.sqrt(1-u1)*math.cos(2*math.pi*u2),
               math.sqrt(u1)*math.sin(2*math.pi*u3), math.sqrt(u1)*math.cos(2*math.pi*u3))


class FreeOrientationDiagnostic(Node):
    def __init__(self):
        super().__init__("baseline_a_free_orientation_diagnostic")
        self.declare_parameter("config_file", "")
        path = Path(self.get_parameter("config_file").value)
        with path.open(encoding="utf-8") as stream: self.cfg = yaml.safe_load(stream)
        self.a, self.c = self.cfg["baseline_a"], self.cfg["head_camera"]
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.valid_client = self.create_client(GetStateValidity, "/check_state_validity")

    def call(self, client, request):
        future = client.call_async(request); rclpy.spin_until_future_complete(self, future)
        if future.result() is None: raise RuntimeError(client.srv_name)
        return future.result()

    def home_state(self):
        state = RobotState(); state.joint_state = JointState(name=LEFT+RIGHT, position=LH+RH)
        return state

    def target(self, distance):
        deadline = self.get_clock().now() + Duration(seconds=5)
        while not self.tf.can_transform(self.a["planning_frame"], self.c["optical_frame"], rclpy.time.Time()):
            if self.get_clock().now() > deadline:
                raise RuntimeError("camera optical TF unavailable")
            rclpy.spin_once(self, timeout_sec=.1)
        tr = self.tf.lookup_transform(self.a["planning_frame"], self.c["optical_frame"], rclpy.time.Time(), timeout=Duration(seconds=5))
        q = tr.transform.rotation; cq = (q.x, q.y, q.z, q.w)
        d = rotate((0., 0., distance), cq); t = tr.transform.translation
        pose = PoseStamped(); pose.header.frame_id = self.a["planning_frame"]
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x+d[0], t.y+d[1], t.z+d[2]
        return pose, (t.x, t.y, t.z), cq

    def try_ik(self, pose, seed):
        request = GetPositionIK.Request(); request.ik_request = PositionIKRequest()
        ik = request.ik_request; ik.group_name = "left_arm"; ik.robot_state = seed; ik.avoid_collisions = False; ik.pose_stamped = pose
        ik.timeout.nanosec = 250000000
        result = self.call(self.ik_client, request)
        return result.solution if result.error_code.val == 1 else None

    def valid(self, solution):
        values = dict(zip(solution.joint_state.name, solution.joint_state.position)); values.update(zip(RIGHT, RH))
        state = self.home_state(); state.joint_state.position = [values.get(n, v) for n, v in zip(LEFT+RIGHT, LH+RH)]
        request = GetStateValidity.Request(); request.robot_state = state; request.group_name = "bimanual_arms"
        return self.call(self.valid_client, request).valid, state

    def run(self):
        for c in (self.ik_client, self.valid_client):
            if not c.wait_for_service(timeout_sec=15): raise RuntimeError(f"missing {c.srv_name}")
        # 24 deterministic SO(3) samples: coarse existence diagnostic, not an
        # optimization. Increase only in a dedicated longer diagnostic run.
        home = self.home_state(); sample_count = 24
        for distance in (.20, .25, .30, .35):
            pose, camera_xyz, camera_q = self.target(distance); ik_count = valid_count = 0; examples = []
            for index, q in enumerate(uniform_quaternions(sample_count)):
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
                solution = self.try_ik(pose, home)
                if not solution:
                    self.get_logger().info(f"FREE d={distance:.2f} i={index:02d}: NO_IK_SOLUTION")
                    continue
                ik_count += 1; is_valid, state = self.valid(solution)
                if not is_valid:
                    self.get_logger().info(f"FREE d={distance:.2f} i={index:02d}: IK_FOUND_BUT_COLLISION_INVALID")
                    continue
                valid_count += 1
                values = dict(zip(state.joint_state.name, state.joint_state.position))
                cost = math.sqrt(sum((values[n]-h)**2 for n,h in zip(LEFT,LH)))
                z_dir = rotate((0.,0.,1.), q)
                self.get_logger().info(f"FREE d={distance:.2f} i={index:02d}: IK_FOUND_AND_VALID q={q} tcp_z={z_dir} home_cost={cost:.6f} left={[values[n] for n in LEFT]}")
                if len(examples) < 3: examples.append((q, cost, z_dir))
            verdict = "POSITION_REACHABLE" if valid_count else ("POSITION_IK_REACHABLE_BUT_COLLISION_BLOCKED" if ik_count else "POSITION_NOT_PROVEN_REACHABLE")
            self.get_logger().info(f"FREE SUMMARY d={distance:.2f}: tested={sample_count} ik_found={ik_count} collision_valid={valid_count} verdict={verdict} target_xyz=({pose.pose.position.x:.6f},{pose.pose.position.y:.6f},{pose.pose.position.z:.6f})")
            # Source URDF: left_link0 origin in body is (0,.031,.505), R_x(-pi/2).
            bx, by, bz = pose.pose.position.x+0.17858, pose.pose.position.y-0.24336, pose.pose.position.z+0.00468
            sx, sy, sz = bx, -(bz-.505), by-.031
            self.get_logger().info(f"GEOMETRY d={distance:.2f}: body_relative=({bx:.6f},{by:.6f},{bz:.6f}) shoulder_relative=({sx:.6f},{sy:.6f},{sz:.6f}) shoulder_distance={math.sqrt(sx*sx+sy*sy+sz*sz):.6f} examples={examples}")


def main():
    rclpy.init(); node = FreeOrientationDiagnostic()
    try: node.run()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()
