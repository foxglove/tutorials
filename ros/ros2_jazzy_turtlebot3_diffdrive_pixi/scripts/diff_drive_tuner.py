#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def move_toward(current: float, target: float, max_delta: float) -> float:
    if max_delta <= 0.0:
        return target
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + max_delta if delta > 0.0 else current - max_delta


class DiffDriveTuner(Node):
    def __init__(self) -> None:
        super().__init__("diff_drive_tuner")

        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("linear_bias", 0.0)
        self.declare_parameter("angular_bias", 0.0)
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 1.8)
        self.declare_parameter("linear_accel_limit", 0.8)
        self.declare_parameter("angular_accel_limit", 2.5)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)

        self._sub = None
        self._pub = None
        self._timer = None

        self._input_topic = ""
        self._output_topic = ""
        self._linear_scale = 1.0
        self._angular_scale = 1.0
        self._linear_bias = 0.0
        self._angular_bias = 0.0
        self._max_linear = 0.4
        self._max_angular = 1.8
        self._linear_accel_limit = 0.8
        self._angular_accel_limit = 2.5
        self._cmd_timeout = 0.5

        self._last_msg_time: Optional[float] = None
        self._last_publish_time: Optional[float] = None
        self._last_output = Twist()

        self._load_params_and_rewire_topics()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"DiffDrive tuner started. input={self._input_topic} output={self._output_topic}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = self._now_s()
        self._last_msg_time = now
        self._publish_smoothed(msg, now)

    def _on_timer(self) -> None:
        now = self._now_s()
        if self._cmd_timeout <= 0.0 or self._last_msg_time is None:
            return
        if (now - self._last_msg_time) < self._cmd_timeout:
            return
        zero = Twist()
        self._publish_smoothed(zero, now)

    def _publish_smoothed(self, msg: Twist, now: float) -> None:
        raw_linear = msg.linear.x * self._linear_scale + self._linear_bias
        raw_angular = msg.angular.z * self._angular_scale + self._angular_bias

        target_linear = clamp(raw_linear, -self._max_linear, self._max_linear)
        target_angular = clamp(raw_angular, -self._max_angular, self._max_angular)

        dt = 0.0
        if self._last_publish_time is not None:
            dt = max(0.0, now - self._last_publish_time)

        if dt > 0.0:
            target_linear = move_toward(
                self._last_output.linear.x,
                target_linear,
                self._linear_accel_limit * dt,
            )
            target_angular = move_toward(
                self._last_output.angular.z,
                target_angular,
                self._angular_accel_limit * dt,
            )

        out = Twist()
        out.linear.x = target_linear
        out.angular.z = target_angular
        self._pub.publish(out)
        self._last_publish_time = now
        self._last_output = out

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name in {"max_linear_speed", "max_angular_speed"} and param.value <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be > 0",
                )
            if param.name in {"linear_accel_limit", "angular_accel_limit"} and param.value < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be >= 0",
                )
            if param.name == "publish_rate_hz" and param.value <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="publish_rate_hz must be > 0",
                )
            if param.name == "cmd_timeout" and param.value < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="cmd_timeout must be >= 0",
                )

        self._load_params_and_rewire_topics()
        return SetParametersResult(successful=True)

    def _load_params_and_rewire_topics(self) -> None:
        previous_input = self._input_topic
        previous_output = self._output_topic

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._linear_scale = float(self.get_parameter("linear_scale").value)
        self._angular_scale = float(self.get_parameter("angular_scale").value)
        self._linear_bias = float(self.get_parameter("linear_bias").value)
        self._angular_bias = float(self.get_parameter("angular_bias").value)
        self._max_linear = float(self.get_parameter("max_linear_speed").value)
        self._max_angular = float(self.get_parameter("max_angular_speed").value)
        self._linear_accel_limit = float(self.get_parameter("linear_accel_limit").value)
        self._angular_accel_limit = float(self.get_parameter("angular_accel_limit").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        if self._pub is None or previous_output != self._output_topic:
            if self._pub is not None:
                self.destroy_publisher(self._pub)
            self._pub = self.create_publisher(Twist, self._output_topic, 10)

        if self._sub is None or previous_input != self._input_topic:
            if self._sub is not None:
                self.destroy_subscription(self._sub)
            self._sub = self.create_subscription(Twist, self._input_topic, self._on_cmd_vel, 10)

        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main() -> None:
    rclpy.init()
    node = DiffDriveTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
