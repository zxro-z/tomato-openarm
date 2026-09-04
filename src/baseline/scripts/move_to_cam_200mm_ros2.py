#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

CAMERA_FORWARD_DISTANCE_M = 0.20

# Temporary offsets applied after projecting the camera-frame target into
# `right_base_link`. Remove or zero these after final camera calibration.
TEMP_TARGET_OFFSET_X_M = -0.15
TEMP_TARGET_OFFSET_Y_M = -0.10
TEMP_TARGET_OFFSET_Z_M = 0.00

SWING_ANGLE_START_DEG = 90
SWING_ANGLE_END_DEG = -90
SWING_ANGLE_STEP_DEG = -15
IK_TIMEOUT_SEC = 3.0
TRAJECTORY_DURATION_SEC = 3.0

R_HORIZONTAL = np.array([
    [0.0,  0.0, 1.0],
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
])

class PiperMoveToObservation(Node):
    def __init__(self):
        super().__init__('piper_baseline_a_step2')
        
        # [SRDF 명칭 적용] 그룹, 기준 링크, 끝단 링크[cite: 1]
        self.group_name = 'right_arm'        #[cite: 1]
        self.base_frame = 'right_base_link'
        self.ee_link = 'right_gripper_base'         #[cite: 1]
        
        # 1. MoveIt IK 서비스 클라이언트
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 2. 확인된 정확한 액션 서버 이름으로 클라이언트 생성
        self.traj_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/right_arm_controller/follow_joint_trajectory'
        )
        
        # 기존 코드 아래에 RViz 직접 시각화용 퍼블리셔 추가
        self.rviz_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info("ROS 2 Piper [2. Move] 최종 실행 노드가 초기화되었습니다.")

    def get_target_pose_matrix(self, dist_m, pitch_deg, yaw_deg, roll_deg):
        """가변 거리(dist_m) 및 3축 회전 각도를 적용한 목표 행렬을 반환합니다."""
        T_base_cam = np.array([
            [-0.990600,  0.027662,  0.133963,  0.279143],
            [-0.009411,  0.963238, -0.268486,  0.218154],
            [-0.136465, -0.267223, -0.953923,  0.542246],
            [ 0.000000,  0.000000,  0.000000,  1.000000]
        ])

        rot_facing_cam = R.from_euler('yzx', [pitch_deg, yaw_deg, roll_deg], degrees=True).as_matrix()
        
        T_cam_target = np.identity(4)
        T_cam_target[:3, :3] = rot_facing_cam
        
        # 다시 Z축(카메라 정면) 거리 하나만 설정하도록 복구
        T_cam_target[:3, 3] = [0.0, 0.0, dist_m]

        return np.dot(T_base_cam, T_cam_target)

    def apply_temporary_target_offsets(self, target_matrix):
        """임시 보정 오프셋을 base frame 좌표에 적용합니다."""
        target_matrix[0, 3] += TEMP_TARGET_OFFSET_X_M
        target_matrix[1, 3] += TEMP_TARGET_OFFSET_Y_M
        target_matrix[2, 3] += TEMP_TARGET_OFFSET_Z_M
        return target_matrix

    def execute_move(self):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("'/compute_ik' 서비스가 없습니다! launch 파일 실행 여부를 확인하세요.")
            return

        self.get_logger().info("카메라 정면 약 20cm 목표 지점의 IK를 직접 계산합니다...")

        best_target_positions = None
        best_arm_joint_names = None

        for swing_angle_deg in range(
            SWING_ANGLE_START_DEG,
            SWING_ANGLE_END_DEG - 1,
            SWING_ANGLE_STEP_DEG,
        ):
            self.get_logger().info(
                f"툴 로컬 X축 swing {swing_angle_deg}도로 IK 솔루션 탐색 중..."
            )

            req = GetPositionIK.Request()

            ik_req = PositionIKRequest()
            ik_req.group_name = self.group_name
            ik_req.ik_link_name = self.ee_link
            ik_req.avoid_collisions = True
            ik_req.timeout = Duration(seconds=IK_TIMEOUT_SEC).to_msg()

            rs = RobotState()
            rs.is_diff = True
            ik_req.robot_state = rs

            ik_req.pose_stamped = PoseStamped()
            ik_req.pose_stamped.header.frame_id = self.base_frame
            ik_req.pose_stamped.header.stamp = self.get_clock().now().to_msg()

            target_matrix = self.get_target_pose_matrix(
                dist_m=CAMERA_FORWARD_DISTANCE_M,
                pitch_deg=0.0,
                yaw_deg=0.0,
                roll_deg=0.0,
            )
            target_matrix = self.apply_temporary_target_offsets(target_matrix)

            # 수평 기준 자세에 대해 툴 로컬 X축 swing을 후적용합니다.
            R_swing = R.from_euler('x', swing_angle_deg, degrees=True).as_matrix()
            target_matrix[:3, :3] = np.dot(R_HORIZONTAL, R_swing)

            ik_req.pose_stamped.pose.position.x = float(target_matrix[0, 3])
            ik_req.pose_stamped.pose.position.y = float(target_matrix[1, 3])
            ik_req.pose_stamped.pose.position.z = float(target_matrix[2, 3])

            self.get_logger().info(
                f"계산된 목표 좌표: X={target_matrix[0, 3]:.3f}, "
                f"Y={target_matrix[1, 3]:.3f}, Z={target_matrix[2, 3]:.3f}"
            )

            rot_matrix = target_matrix[:3, :3]
            quat = R.from_matrix(rot_matrix).as_quat()

            ik_req.pose_stamped.pose.orientation.x = float(quat[0])
            ik_req.pose_stamped.pose.orientation.y = float(quat[1])
            ik_req.pose_stamped.pose.orientation.z = float(quat[2])
            ik_req.pose_stamped.pose.orientation.w = float(quat[3])

            req.ik_request = ik_req

            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            if res.error_code.val == 1:
                self.get_logger().info(
                    f"★ swing {swing_angle_deg}도에서 IK 솔루션 발견 성공!"
                )

                arm_joint_names = []
                target_positions = []

                for i in range(1, 7):
                    joint_name = f"right_joint{i}"
                    if joint_name in res.solution.joint_state.name:
                        idx = res.solution.joint_state.name.index(joint_name)
                        arm_joint_names.append(joint_name)
                        target_positions.append(res.solution.joint_state.position[idx])

                best_arm_joint_names = arm_joint_names
                best_target_positions = target_positions
                break

        if best_target_positions is None:
            self.get_logger().error(
                "모든 swing 각도에서 IK 계산에 실패했습니다. 목표 지점이 도달 범위를 벗어났습니다."
            )
            return

        self.get_logger().info(f"최종 목표 관절 각도(rad): {np.round(best_target_positions, 4)}")

        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = best_arm_joint_names
        js_msg.position = best_target_positions
        self.rviz_pub.publish(js_msg)

        self.get_logger().info("액션 서버 '/right_arm_controller/follow_joint_trajectory' 연결을 기다리는 중...")
        while not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("  -> 서버 연결 대기 중... (RViz / 컨트롤러 노드가 켜져 있는지 확인 중)")

        self.get_logger().info("★ 액션 서버 연결 성공!")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = best_arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = best_target_positions
        point.time_from_start = Duration(seconds=TRAJECTORY_DURATION_SEC).to_msg()
        goal_msg.trajectory.points.append(point)

        self.get_logger().info("로봇에 이동 명령(Goal)을 전송합니다...")
        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("로봇 컨트롤러가 이동 명령을 거부(Reject)했습니다!")
            return

        self.get_logger().info("이동 명령 승인됨! 로봇이 최적의 관측 지점으로 이동 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == 0:
            self.get_logger().info("===== [2. Move 완료] 카메라 앞 최적 작업 위치 도달 성공! =====")
        else:
            self.get_logger().error(f"궤적 이동 중 오류 발생! Error Code: {result.error_code}")

def main(args=None):
    rclpy.init(args=args)
    node = PiperMoveToObservation()
    try:
        node.execute_move()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
