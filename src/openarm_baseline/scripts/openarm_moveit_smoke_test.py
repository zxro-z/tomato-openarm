#!/usr/bin/env python3
import copy
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState


LEFT_HOME = [
    0.349065850399,
    0.0,
    0.0,
    1.047197551197,
    0.0,
    0.0,
    -0.767944870878,
]
RIGHT_HOME = [
    -0.349065850399,
    0.0,
    0.0,
    1.047197551197,
    0.0,
    0.0,
    0.767944870878,
]

LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]


class SmokeTestNode(Node):
    def __init__(self):
        super().__init__("openarm_moveit_smoke_test")
        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_cli = self.create_client(GetStateValidity, "/check_state_validity")

    def wait_for_services(self):
        for client in (self.fk_cli, self.ik_cli, self.validity_cli):
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"Service unavailable: {client.srv_name}")

    def make_robot_state(self):
        joint_state = JointState()
        joint_state.name = LEFT_JOINTS + RIGHT_JOINTS
        joint_state.position = LEFT_HOME + RIGHT_HOME
        state = RobotState()
        state.joint_state = joint_state
        return state

    def fk(self, state):
        request = GetPositionFK.Request()
        request.header.frame_id = "openarm_body_link0"
        request.fk_link_names = ["openarm_right_hand_tcp"]
        request.robot_state = state
        future = self.fk_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def ik(self, pose, seed_state):
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.robot_state = seed_state
        request.ik_request.avoid_collisions = True
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout.sec = 1
        future = self.ik_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def state_validity(self, state):
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = "right_arm"
        request.constraints = Constraints(
            joint_constraints=[
                JointConstraint(joint_name=name, position=value, tolerance_above=0.0, tolerance_below=0.0, weight=1.0)
                for name, value in zip(LEFT_JOINTS, LEFT_HOME)
            ]
        )
        future = self.validity_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = SmokeTestNode()
    try:
        node.wait_for_services()
        seed_state = node.make_robot_state()

        fk_response = node.fk(seed_state)
        if not fk_response or not fk_response.pose_stamped:
            raise RuntimeError("FK failed for openarm_right_hand_tcp")
        pose = fk_response.pose_stamped[0]
        node.get_logger().info(f"FK openarm_right_hand_tcp pose: {pose}")

        ik_pose = PoseStamped()
        ik_pose.header.frame_id = "openarm_body_link0"
        ik_pose.pose = copy.deepcopy(pose.pose)
        ik_pose.pose.position.x += 0.02
        ik_response = node.ik(ik_pose, seed_state)
        if not ik_response or ik_response.error_code.val != 1:
            raise RuntimeError(f"IK failed with code {ik_response.error_code.val if ik_response else 'None'}")

        solution = dict(zip(ik_response.solution.joint_state.name, ik_response.solution.joint_state.position))
        left_after = [solution.get(name, LEFT_HOME[i]) for i, name in enumerate(LEFT_JOINTS)]
        right_after = [solution[name] for name in RIGHT_JOINTS]

        node.get_logger().info(f"Left joints before: {LEFT_HOME}")
        node.get_logger().info(f"Left joints after : {left_after}")
        node.get_logger().info(f"Right joints before: {RIGHT_HOME}")
        node.get_logger().info(f"Right joints after : {right_after}")

        unchanged = all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(left_after, LEFT_HOME))
        right_changed = any(not math.isclose(a, b, abs_tol=1e-6) for a, b in zip(right_after, RIGHT_HOME))
        if not unchanged:
            raise RuntimeError("Left arm changed during right-arm IK.")
        if not right_changed:
            raise RuntimeError("Right arm did not move in IK solution.")

        validity = node.state_validity(ik_response.solution)
        if not validity or not validity.valid:
            raise RuntimeError("IK solution is not collision-free / state-valid.")

        node.get_logger().info("Smoke test passed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
