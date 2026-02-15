#!/usr/bin/env python3
"""LIP control node for ROS 2."""

from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LIPController(Node):
    """LIP Control."""

    def __init__(self):
        super().__init__('lip_control')

        self.declare_parameter('joint_name', 'j_0')
        self.declare_parameter('kp', 220.0)
        self.declare_parameter('ki', 5.0)
        self.declare_parameter('kd', 40.0)
        self.declare_parameter('target_theta', 0.0)
        self.declare_parameter('effort_limit', 120.0)
        self.declare_parameter('integral_limit', 30.0)
        self.declare_parameter('control_sign', 1.0)

        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.target_theta = self.get_parameter('target_theta').get_parameter_value().double_value
        self.effort_limit = self.get_parameter('effort_limit').get_parameter_value().double_value
        self.integral_limit = self.get_parameter('integral_limit').get_parameter_value().double_value
        self.control_sign = self.get_parameter('control_sign').get_parameter_value().double_value

        self.integral = 0.0
        self.last_update = self.get_clock().now()
        self.joint_index = None

        self.command_topics = [
            '/lip/joint_effort_controller_j_0/commands',
            '/joint_effort_controller_j_0/commands',
        ]
        self.command_publishers = [
            self.create_publisher(Float64MultiArray, topic, 10)
            for topic in self.command_topics
        ]

        self.joint_state_topics = ['/lip/joint_states', '/joint_states']
        self.joint_state_subscriptions = []
        for topic in self.joint_state_topics:
            self.joint_state_subscriptions.append(
                self.create_subscription(JointState, topic, self.callback, 10)
            )

        self.get_logger().info(
            f"LIP PID started. Reading joint states from {self.joint_state_topics} and publishing efforts to {self.command_topics}"
        )

    @staticmethod
    def wrap_to_pi(angle: float) -> float:
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def callback(self, data: JointState) -> None:
        if not data.position or not data.name:
            return

        if self.joint_index is None:
            if self.joint_name in data.name:
                self.joint_index = data.name.index(self.joint_name)
            else:
                fallback_index = None
                for idx, name in enumerate(data.name):
                    if name.endswith(self.joint_name):
                        fallback_index = idx
                        break
                if fallback_index is not None:
                    self.joint_index = fallback_index

            if self.joint_index is None:
                self.get_logger().warn(
                    f"Joint '{self.joint_name}' not in JointState names: {list(data.name)}"
                )
                return
            self.get_logger().info(f"Controlling joint '{self.joint_name}' at index {self.joint_index}")

        if self.joint_index >= len(data.position):
            return

        theta = data.position[self.joint_index]
        theta_dot = 0.0
        if self.joint_index < len(data.velocity):
            theta_dot = data.velocity[self.joint_index]

        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now
        if dt <= 0.0:
            dt = 0.001

        error = self.wrap_to_pi(self.target_theta - theta)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        effort = self.control_sign * (
            (self.kp * error) +
            (self.ki * self.integral) -
            (self.kd * theta_dot)
        )
        effort = max(min(effort, self.effort_limit), -self.effort_limit)

        command_msg = Float64MultiArray()
        command_msg.data = [effort]
        for publisher in self.command_publishers:
            publisher.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = LIPController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
