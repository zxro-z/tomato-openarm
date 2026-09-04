#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINT_NAMES = [
    'right_joint1',
    'right_joint2',
    'right_joint3',
    'right_joint4',
    'right_joint5',
    'right_joint6',
]

J6_JOINT_NAME = 'right_joint6'
J6_URDF_MIN_RAD = -2.0944
J6_URDF_MAX_RAD = 2.0944
J6_URDF_MIN_DEG = math.degrees(J6_URDF_MIN_RAD)
J6_URDF_MAX_DEG = math.degrees(J6_URDF_MAX_RAD)

BASELINE_J6_MIN_DEG = -170.0
BASELINE_J6_MAX_DEG = 170.0
J6_STEP_DEG = 10.0

MOVE_DURATION_SEC = 2.0
SETTLE_TIME_SEC = 0.5
RETURN_TO_START = False


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def build_waypoint_degrees():
    effective_min_deg = clamp(BASELINE_J6_MIN_DEG, J6_URDF_MIN_DEG, J6_URDF_MAX_DEG)
    effective_max_deg = clamp(BASELINE_J6_MAX_DEG, J6_URDF_MIN_DEG, J6_URDF_MAX_DEG)

    waypoints_deg = []
    current_deg = effective_min_deg
    while current_deg <= effective_max_deg + 1e-9:
        waypoints_deg.append(round(current_deg, 6))
        current_deg += J6_STEP_DEG

    if not waypoints_deg or abs(waypoints_deg[-1] - effective_max_deg) > 1e-6:
        waypoints_deg.append(round(effective_max_deg, 6))

    return waypoints_deg, effective_min_deg, effective_max_deg


class BaselineBObserveJ6Sweep(Node):
    def __init__(self):
        super().__init__('baseline_b_observe_j6')

        self.current_arm_joint_positions = None
        self.current_joint_state_stamp = None

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.get_logger().info('Baseline B Observe J6 sweep 노드가 초기화되었습니다.')

    def joint_state_callback(self, msg):
        positions_by_name = dict(zip(msg.name, msg.position))
        if not all(joint_name in positions_by_name for joint_name in ARM_JOINT_NAMES):
            return

        self.current_arm_joint_positions = [
            float(positions_by_name[joint_name]) for joint_name in ARM_JOINT_NAMES
        ]
        self.current_joint_state_stamp = msg.header.stamp

    def wait_for_current_arm_state(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_arm_joint_positions is not None:
                return list(self.current_arm_joint_positions)
        return None

    def wait_for_action_server(self):
        self.get_logger().info("액션 서버 '/right_arm_controller/follow_joint_trajectory' 연결을 기다리는 중...")
        while not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('  -> 서버 연결 대기 중...')

    def send_joint_goal(self, target_positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(ARM_JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.time_from_start = Duration(seconds=MOVE_DURATION_SEC).to_msg()
        goal_msg.trajectory.points.append(point)

        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            return False, 'goal_rejected'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            return False, 'no_result'
        if result.result.error_code != 0:
            return False, f'error_code_{result.result.error_code}'
        return True, 'success'

    def on_observation_waypoint(self, j6_angle_deg):
        # TODO: camera capture / marker detection
        _ = j6_angle_deg

    def execute_sweep(self):
        start_positions = self.wait_for_current_arm_state()
        if start_positions is None:
            self.get_logger().error("'/joint_states'에서 현재 arm joint state를 읽지 못했습니다.")
            return

        self.wait_for_action_server()

        start_j6_deg = math.degrees(start_positions[5])
        waypoints_deg, effective_min_deg, effective_max_deg = build_waypoint_degrees()

        self.get_logger().info(
            'Baseline B Observe sweep 시작: '
            f'현재 J6={start_j6_deg:.2f} deg, '
            f'실행 범위=[{effective_min_deg:.2f}, {effective_max_deg:.2f}] deg'
        )

        if effective_min_deg != BASELINE_J6_MIN_DEG or effective_max_deg != BASELINE_J6_MAX_DEG:
            self.get_logger().warn(
                'Baseline 요청 범위 -170~170 deg가 URDF J6 limit을 벗어나므로 '
                f'실제 sweep은 [{effective_min_deg:.2f}, {effective_max_deg:.2f}] deg로 제한됩니다.'
            )

        for index, target_j6_deg in enumerate(waypoints_deg, start=1):
            target_positions = list(start_positions)
            target_positions[5] = math.radians(target_j6_deg)

            self.get_logger().info(
                f'[Baseline B] waypoint {index}/{len(waypoints_deg)} : '
                f'J6 = {target_j6_deg:.1f} deg'
            )

            success, status = self.send_joint_goal(target_positions)
            if not success:
                self.get_logger().error(
                    f'waypoint {index} 이동 실패: {status}'
                )
                return

            self.on_observation_waypoint(target_j6_deg)
            time.sleep(SETTLE_TIME_SEC)

        if RETURN_TO_START:
            self.get_logger().info(
                f'시작 J6 위치 {start_j6_deg:.2f} deg로 복귀합니다.'
            )
            success, status = self.send_joint_goal(start_positions)
            if not success:
                self.get_logger().error(f'시작 자세 복귀 실패: {status}')
                return

        self.get_logger().info('===== [Baseline B Observe] J6 sweep 완료 =====')


def main(args=None):
    rclpy.init(args=args)
    node = BaselineBObserveJ6Sweep()
    try:
        node.execute_sweep()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
