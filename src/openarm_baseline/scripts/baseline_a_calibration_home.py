#!/usr/bin/python3
"""Prepare calibration RViz: bimanual HOME, then left gripper open."""
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from baseline_a_single_view import A0SingleView, LEFT_HOME, RIGHT_HOME, LEFT_JOINTS, RIGHT_JOINTS


def main():
    rclpy.init(); node=A0SingleView()
    # The contact-grasp node is intentionally launched after fake controllers.
    # Latch readiness so a normal launch-timing variation cannot lose the gate.
    ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ready=node.create_publisher(Bool,'baseline_a/grasp_calibration_open_ready',ready_qos)
    try:
        with Path(node.get_parameter('config_file').value).open(encoding='utf-8') as stream:
            grasp = yaml.safe_load(stream)['baseline_a']['grasp']
        gripper = ActionClient(node, FollowJointTrajectory,
                               '/left_gripper_controller/follow_joint_trajectory')
        node.wait_ready()
        if not gripper.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('left_gripper_controller action unavailable')
        start=node.robot_state()
        if not node.valid(start,'calibration current start'): raise RuntimeError('GRASP_CALIBRATION_HOME_FAILED: invalid start')
        # GenericSystem now publishes the canonical HOME as its first arm
        # state.  Preserve this node as the readiness/validation gate, but do
        # not send a redundant zero-length bimanual trajectory at startup.
        try:
            node.validate_bimanual_home()
            node.get_logger().info('CALIBRATION_START_ALREADY_HOME: no arm trajectory sent')
        except RuntimeError as error:
            node.get_logger().warn(f'CALIBRATION_START_NOT_HOME: {error}; planning bimanual HOME recovery')
            home=node.plan(start,node.joint_goal(LEFT_JOINTS+RIGHT_JOINTS,LEFT_HOME+RIGHT_HOME),'calibration bimanual HOME recovery',group='bimanual_arms')
            node.execute(home,'calibration bimanual HOME recovery')
            node.validate_bimanual_home()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = grasp['calibration_open_joint_names']
        point = JointTrajectoryPoint(); point.positions = grasp['calibration_open_joint_values_m']
        point.time_from_start.sec = 1; goal.trajectory.points = [point]
        future = gripper.send_goal_async(goal); rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('calibration gripper-open goal rejected')
        result = handle.get_result_async(); rclpy.spin_until_future_complete(node, result)
        if result.result().result.error_code != 0:
            raise RuntimeError(f'calibration gripper-open failed code={result.result().result.error_code}')
        for _ in range(10): rclpy.spin_once(node, timeout_sec=.05)
        errors=[]
        for name, expected in zip(grasp['calibration_open_joint_names'], grasp['calibration_open_joint_values_m']):
            actual=node.current_joints[name]; error=abs(actual-expected); errors.append(error)
            node.get_logger().info(f'CALIBRATION_OPEN joint {name}: actual={actual:.9f} expected={expected:.9f} abs_error={error:.9f}')
        if any(error > grasp['calibration_open_tolerance_m'] for error in errors):
            raise RuntimeError('CALIBRATION_GRIPPER_OPEN_VALIDATION_FAILED')
        node.get_logger().info('CALIBRATION_GRIPPER_OPEN_VALIDATED: left gripper is open')
        ready.publish(Bool(data=True)); node.get_logger().info('GRASP_CALIBRATION_OPEN_READY: HOME and open gripper verified')
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Normal launch shutdown after HOME is not a HOME validation failure.
        pass
    except Exception as error:
        node.get_logger().error(f'GRASP_CALIBRATION_HOME_FAILED: {error}')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
