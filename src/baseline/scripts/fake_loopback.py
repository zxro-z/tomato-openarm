import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FixedLoopback(Node):
    def __init__(self):
        super().__init__('fixed_loopback_node')
        self.sub = self.create_subscription(JointState, '/joint_ctrl_cmd', self.cmd_callback, 10)
        self.pub_piper = self.create_publisher(JointState, '/joint_states_piper', 10)
        self.pub_rviz = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('★ 프리픽스가 정비된 가상 루프백 노드가 실행되었습니다!')

    def cmd_callback(self, msg):
        # MoveIt 설정에 맞게 right_ 프리픽스 보정
        new_names = []
        for name in msg.name:
            if not name.startswith('right_'):
                new_names.append(f'right_{name}')
            else:
                new_names.append(name)
        
        msg.name = new_names
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 상태 토픽 발행
        self.pub_piper.publish(msg)
        self.pub_rviz.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(FixedLoopback())
    rclpy.shutdown()

if __name__ == '__main__':
    main()