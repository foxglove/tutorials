#!/usr/bin/env python3
"""Converts Twist "jog" commands (e.g. from Foxglove's Teleop panel) into
joint-angle targets for the arm's JointPositionController cmd_pos topics.

The Teleop panel always publishes geometry_msgs/Twist-shaped messages, so it
can't target the Float64 cmd_pos topics directly. Instead, two Twist topics
feed this node, which integrates them into an absolute joint angle (clamped
to the joint's limits) and republishes that to the matching cmd_pos topic --
only while a button is actually held, since a message on a Twist topic times
out and its velocity is treated as zero after STALE_TIMEOUT_SEC.

  /foxglove_arm/teleop/shoulder  linear.x -> shoulder_tilt_joint
                                  linear.y -> shoulder_pan_joint
  /foxglove_arm/teleop/wrist     linear.x -> wrist_tilt_joint

Don't run this alongside move_viewpoints.py or manual `ros2 topic pub` to the
same cmd_pos topics -- they'll fight over the same joints.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

TIMER_HZ = 20.0
DT = 1.0 / TIMER_HZ
STALE_TIMEOUT_SEC = 0.15
SPEED_RAD_PER_SEC = 1.0  # per unit of Twist linear value

JOINT_LIMITS = {
    "shoulder_pan_joint": (-3.14, 3.14),
    "shoulder_tilt_joint": (-1.57, 1.57),
    "wrist_tilt_joint": (-1.9, 1.9),
}


class TeleopArmBridge(Node):
    def __init__(self):
        super().__init__("teleop_arm_bridge")

        self._angle = {joint_name: 0.0 for joint_name in JOINT_LIMITS}
        self._cmd_pub = {
            joint_name: self.create_publisher(
                Float64, f"/foxglove_arm/{joint_name}/cmd_pos", 10
            )
            for joint_name in JOINT_LIMITS
        }

        self._shoulder_twist = Twist()
        self._wrist_twist = Twist()
        self._last_shoulder_msg = self.get_clock().now()
        self._last_wrist_msg = self.get_clock().now()

        self.create_subscription(
            Twist, "/foxglove_arm/teleop/shoulder", self._on_shoulder, 10
        )
        self.create_subscription(
            Twist, "/foxglove_arm/teleop/wrist", self._on_wrist, 10
        )
        self.create_timer(DT, self._on_timer)

    def _on_shoulder(self, msg):
        self._shoulder_twist = msg
        self._last_shoulder_msg = self.get_clock().now()

    def _on_wrist(self, msg):
        self._wrist_twist = msg
        self._last_wrist_msg = self.get_clock().now()

    def _seconds_since(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _on_timer(self):
        shoulder_active = self._seconds_since(self._last_shoulder_msg) <= STALE_TIMEOUT_SEC
        wrist_active = self._seconds_since(self._last_wrist_msg) <= STALE_TIMEOUT_SEC

        if shoulder_active:
            self._integrate("shoulder_tilt_joint", self._shoulder_twist.linear.x)
            self._integrate("shoulder_pan_joint", self._shoulder_twist.linear.y)
        if wrist_active:
            self._integrate("wrist_tilt_joint", self._wrist_twist.linear.x)

    def _integrate(self, joint_name, twist_value):
        velocity = twist_value * SPEED_RAD_PER_SEC
        if velocity == 0.0:
            return
        lower, upper = JOINT_LIMITS[joint_name]
        angle = max(lower, min(upper, self._angle[joint_name] + velocity * DT))
        self._angle[joint_name] = angle

        msg = Float64()
        msg.data = angle
        self._cmd_pub[joint_name].publish(msg)


def main():
    rclpy.init()
    node = TeleopArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
