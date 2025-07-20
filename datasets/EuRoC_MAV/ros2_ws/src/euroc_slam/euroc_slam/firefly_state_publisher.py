#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class StatePublisher(Node):
    def __init__(self):
        super().__init__('firefly_state_publisher')

        # 1) spin_rate in revolutions per second
        self.declare_parameter('spin_rate', 10.0)
        self.spin_rate = self.get_parameter('spin_rate').get_parameter_value().double_value

        # 2) We’ll update at 30 Hz
        self.update_hz = 30.0
        period = 1.0 / self.update_hz

        # 3) Precompute angular velocity (rad/s) and per‑tick increment
        self.ang_vel = 2 * math.pi * self.spin_rate      # rad/s
        self.increment = self.ang_vel * period           # rad per update

        # 4) JointState publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state = JointState()
        self.joint_state.name = [
            'firefly_rotor_0_joint',
            'firefly_rotor_1_joint',
            'firefly_rotor_2_joint',
            'firefly_rotor_3_joint',
            'firefly_rotor_4_joint',
            'firefly_rotor_5_joint',
        ]

        self.angle = 0.0  # current angle in radians

        # 5) Timer instead of manual loop
        self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'StatePublisher started: spin_rate={self.spin_rate} Hz, update={self.update_hz} Hz')

    def timer_callback(self):
        # advance angle and wrap to [0, 2π)
        self.angle = (self.angle + self.increment) % (2 * math.pi)

        # stamp and publish
        now = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = now

        # alternate spin direction on adjacent rotors
        positions = []
        for i in range(len(self.joint_state.name)):
            sign = 1.0 if (i % 2 == 0) else -1.0
            positions.append(sign * self.angle)
        self.joint_state.position = positions

        self.joint_pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
