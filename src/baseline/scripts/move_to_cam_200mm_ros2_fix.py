#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class PiperMoveToObservation(Node):
    def __init__(self):
        super().__init__('piper_baseline_a_step2')
        
        # [SRDF 명칭 적용] 그룹, 기준 링크, 끝단 링크[cite: 1]
        self.group_name = 'right_arm'        #[cite: 1]
        self.base_frame = 'right_base_link'  #[cite: 1]
        self.ee_link = 'right_link6'         #[cite: 1]
        
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
        T_cam_target[:3, 3] = [0.0, 0.0, dist_m]

        return np.dot(T_base_cam, T_cam_target)

    def execute_move(self):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("'/compute_ik' 서비스가 없습니다! launch 파일 실행 여부를 확인하세요.")
            return

        self.get_logger().info("로봇 정면 기준 안전 관측 지점의 IK를 직접 계산합니다...")

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.robot_state.is_diff = True
        
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        # --- [로봇 베이스 좌표계 기준 직접 지정] ---
        # 로봇 정면 25cm 앞, 높이 20cm 지점으로 EE(끝단)가 아래를 바라보게 설정
        req.ik_request.pose_stamped.pose.position.x = 0.25
        req.ik_request.pose_stamped.pose.position.y = 0.0
        req.ik_request.pose_stamped.pose.position.z = 0.20
        
        # 끝단이 정면 아래를 향하는 안정적인 쿼터니언 자세
        req.ik_request.pose_stamped.pose.orientation.x = 0.0
        req.ik_request.pose_stamped.pose.orientation.y = 0.707
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 0.707
        
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(seconds=1.0).to_msg()

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res.error_code.val != 1:
            self.get_logger().error(f"IK 계산 실패! Error Code: {res.error_code.val}")
            return

        self.get_logger().info("★ IK 솔루션 즉시 계산 성공!")

        # --- [SRDF 관절 이름 매핑] ---
        arm_joint_names = []
        target_positions = []
        
        for i in range(1, 7):
            joint_name = f"right_joint{i}"
            if joint_name in res.solution.joint_state.name:
                idx = res.solution.joint_state.name.index(joint_name)
                arm_joint_names.append(joint_name)
                target_positions.append(res.solution.joint_state.position[idx])
            else:
                self.get_logger().error(f"관절 '{joint_name}'을(를) 찾을 수 없습니다.")
                return

        self.get_logger().info(f"목표 관절 각도(rad): {np.round(target_positions, 4)}")

        # --- [RViz 강제 시각화 퍼블리시] ---
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = arm_joint_names
        js_msg.position = target_positions
        self.rviz_pub.publish(js_msg)

        # 이후 액션 서버 전송 및 이동 로직 계속...

        # --- [★ 핵심 개선: DDS 디스커버리 지연을 고려한 무한 대기 로직] ---
        self.get_logger().info("액션 서버 '/right_arm_controller/follow_joint_trajectory' 연결을 기다리는 중...")
        while not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("  -> 서버 연결 대기 중... (RViz / 컨트롤러 노드가 켜져 있는지 확인 중)")
        
        self.get_logger().info("★ 액션 서버 연결 성공!")

        # --- [로봇 이동 제어 명령 전송] ---
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = Duration(seconds=3.0).to_msg()
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