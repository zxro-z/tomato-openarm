#!/usr/bin/python3
"""Non-invasive startup evidence recorder for the Baseline-A fake launch.

This node deliberately sends no command.  It starts before the fake MoveIt
launch so the log can distinguish a real controller trajectory from RViz
rendering a model before its first /joint_states message arrives.
"""
import time
import xml.etree.ElementTree as ET

import rclpy
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


LEFT = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT = [f"openarm_right_joint{i}" for i in range(1, 8)]
FINGERS = [
    "openarm_left_finger_joint1", "openarm_left_finger_joint2",
    "openarm_right_finger_joint1", "openarm_right_finger_joint2",
]
EXPECTED = dict(zip(
    LEFT + RIGHT + FINGERS,
    [0.349065850399, 0.0, 0.0, 1.047197551197, 0.0, 0.0, -0.767944870878,
     -0.349065850399, 0.0, 0.0, 1.047197551197, 0.0, 0.0, 0.767944870878,
     0.044, 0.044, 0.044, 0.044],
))
ARM_JOINTS = LEFT + RIGHT
GOAL_REQUEST = FollowJointTrajectory.Impl.SendGoalService.Request


class StartupAudit(Node):
    def __init__(self):
        super().__init__('baseline_a_startup_audit')
        self.started = time.monotonic()
        self.samples = []
        self.first = None
        self.first_home_time = None
        self.grasp_ready_time = None
        self.goals = []
        self.runtime_urdf_logged = False
        self.description_future = None
        self.create_subscription(JointState, '/joint_states', self.joint_state, 200)
        self.create_subscription(Bool, 'baseline_a/grasp_calibration_open_ready',
                                 self.grasp_ready, 1)
        for controller in ('left_arm_controller', 'right_arm_controller',
                           'left_gripper_controller', 'right_gripper_controller'):
            topic = f'/{controller}/follow_joint_trajectory/_action/goal'
            self.create_subscription(GOAL_REQUEST, topic,
                                     lambda msg, t=topic: self.goal(t, msg), 20)
        self.parameters = self.create_client(GetParameters,
                                             '/controller_manager/get_parameters')
        self.create_timer(0.25, self.poll_runtime_description)
        self.five_second_timer = self.create_timer(5.05, self.report_first_five_seconds)
        self.ten_second_timer = self.create_timer(10.05, self.report_first_ten_seconds)
        self.get_logger().info('STARTUP_AUDIT_ARMED: passive recorder started before fake controllers')

    def elapsed(self):
        return time.monotonic() - self.started

    @staticmethod
    def complete(message):
        received = dict(zip(message.name, message.position))
        return all(name in received for name in EXPECTED)

    def joint_state(self, message):
        if not self.complete(message):
            return
        values = dict(zip(message.name, message.position))
        entry = (self.elapsed(), values, message.header.stamp.sec,
                 message.header.stamp.nanosec)
        if self.first is None:
            self.first = entry
            self.get_logger().info(
                'STARTUP_AUDIT_FIRST_JOINT_STATE t=%.6fs stamp=%d.%09d left=%s right=%s fingers=%s' % (
                    entry[0], entry[2], entry[3],
                    self.format_values(values, LEFT), self.format_values(values, RIGHT),
                    self.format_values(values, FINGERS)))
        self.samples.append(entry)
        if self.first_home_time is None and self.is_home(values):
            self.first_home_time = entry[0]

    def goal(self, topic, request):
        trajectory = request.goal.trajectory
        first = trajectory.points[0].positions if trajectory.points else []
        last = trajectory.points[-1].positions if trajectory.points else []
        publishers = sorted({f'{info.node_namespace}/{info.node_name}'
                             for info in self.get_publishers_info_by_topic(topic)})
        entry = (self.elapsed(), topic, list(trajectory.joint_names), list(first), list(last), publishers)
        self.goals.append(entry)
        self.get_logger().warn(
            'STARTUP_AUDIT_ACTION_GOAL t=%.6fs topic=%s publishers=%s joints=%s first=%s final=%s' % entry)

    def grasp_ready(self, message):
        if message.data and self.grasp_ready_time is None:
            self.grasp_ready_time = self.elapsed()
            self.get_logger().info('STARTUP_AUDIT_GRASP_GATE t=%.6fs' % self.grasp_ready_time)

    def is_home(self, values):
        return max(abs(values[name] - expected) for name, expected in EXPECTED.items()) <= 1e-3

    @staticmethod
    def format_values(values, names):
        return '[' + ', '.join(f'{values[name]:+.9f}' for name in names) + ']'

    def report_first_five_seconds(self):
        self.five_second_timer.cancel()
        window = [sample for sample in self.samples if sample[0] <= 5.0]
        if self.first is None:
            self.get_logger().error('STARTUP_AUDIT_5S: no complete /joint_states sample received')
            return
        self.get_logger().info(
            'STARTUP_AUDIT_5S samples=%d (first_100_captured=%d) first_home_t=%s' %
            (len(window), min(len(window), 100),
             'NONE' if self.first_home_time is None else f'{self.first_home_time:.6f}s'))
        for name in ARM_JOINTS:
            values = [sample[1][name] for sample in window]
            self.get_logger().info(
                'STARTUP_AUDIT_RANGE %s first=%+.9f min=%+.9f max=%+.9f home=%+.9f first_error=%.9f' %
                (name, self.first[1][name], min(values), max(values), EXPECTED[name],
                 abs(self.first[1][name] - EXPECTED[name])))

    def report_first_ten_seconds(self):
        self.ten_second_timer.cancel()
        arm_goals = [goal for goal in self.goals if ('left_arm_controller' in goal[1] or
                                                     'right_arm_controller' in goal[1]) and goal[0] <= 10.0]
        pre_grasp_goals = [goal for goal in arm_goals if self.grasp_ready_time is None or
                           goal[0] < self.grasp_ready_time]
        self.get_logger().info(
            'STARTUP_AUDIT_10S arm_goals=%d arm_goals_before_grasp_gate=%d grasp_gate=%s' %
            (len(arm_goals), len(pre_grasp_goals),
             'NONE' if self.grasp_ready_time is None else f'{self.grasp_ready_time:.6f}s'))
        if not arm_goals:
            self.get_logger().info('STARTUP_AUDIT_CLASSIFICATION: no arm-controller trajectory in first 10 s')

    def poll_runtime_description(self):
        if self.runtime_urdf_logged:
            return
        if self.description_future is None:
            if not self.parameters.service_is_ready():
                self.parameters.wait_for_service(timeout_sec=0.0)
                return
            request = GetParameters.Request()
            request.names = ['robot_description']
            self.description_future = self.parameters.call_async(request)
            return
        if not self.description_future.done():
            return
        try:
            response = self.description_future.result()
            text = response.values[0].string_value
            root = ET.fromstring(text)
            found = {}
            for joint in root.findall('.//ros2_control/joint'):
                name = joint.attrib.get('name')
                if name not in EXPECTED:
                    continue
                state = joint.find("state_interface[@name='position']")
                command = joint.find("command_interface[@name='position']")
                state_param = state.find("param[@name='initial_value']") if state is not None else None
                command_param = command.find("param[@name='initial_value']") if command is not None else None
                found[name] = (
                    None if state_param is None else float(state_param.text),
                    None if command_param is None else float(command_param.text))
            self.get_logger().info(
                'STARTUP_AUDIT_RUNTIME_URDF position_state/command_initial_values=%s' %
                ', '.join(f'{name}={found.get(name)}' for name in ARM_JOINTS + FINGERS))
            self.runtime_urdf_logged = True
        except Exception as error:
            self.get_logger().error(f'STARTUP_AUDIT_RUNTIME_URDF_FAILED: {error}')
            self.runtime_urdf_logged = True


def main():
    rclpy.init()
    node = StartupAudit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
