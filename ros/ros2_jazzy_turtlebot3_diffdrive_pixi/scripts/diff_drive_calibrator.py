#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class DiffDriveCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("diff_drive_calibrator")

        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("wheel_radius_multiplier", 1.0)
        self.declare_parameter("wheel_separation_multiplier", 1.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)

        self._sub = None
        self._pub = None
        self._timer = None

        self._input_topic = ""
        self._output_topic = ""
        self._wheel_radius_multiplier = 1.0
        self._wheel_separation_multiplier = 1.0
        self._cmd_timeout = 0.5

        self._last_msg_time: Optional[float] = None
        self._last_desired_cmd = Twist()

        self._load_params_and_rewire_topics()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            "Diff-drive calibrator started. "
            f"input={self._input_topic} output={self._output_topic}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_msg_time = self._now_s()
        self._last_desired_cmd = msg

    def _on_timer(self) -> None:
        now = self._now_s()

        desired = Twist()
        if self._last_msg_time is not None:
            if self._cmd_timeout <= 0.0 or (now - self._last_msg_time) < self._cmd_timeout:
                desired = self._last_desired_cmd

        corrected = Twist()

        # Command pre-compensation:
        #   v_actual ~= wheel_radius_multiplier * v_cmd
        #   w_actual ~= (wheel_radius_multiplier / wheel_separation_multiplier) * w_cmd
        # so we solve for cmd that achieves desired (v, w).
        corrected.linear.x = desired.linear.x / self._wheel_radius_multiplier
        corrected.angular.z = (
            desired.angular.z
            * self._wheel_separation_multiplier
            / self._wheel_radius_multiplier
        )

        self._pub.publish(corrected)

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name in {"wheel_radius_multiplier", "wheel_separation_multiplier"}:
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be > 0",
                    )
            if param.name == "publish_rate_hz" and float(param.value) <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="publish_rate_hz must be > 0",
                )
            if param.name == "cmd_timeout" and float(param.value) < 0.0:
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
        self._wheel_radius_multiplier = float(
            self.get_parameter("wheel_radius_multiplier").value
        )
        self._wheel_separation_multiplier = float(
            self.get_parameter("wheel_separation_multiplier").value
        )
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)

        if self._pub is None or previous_output != self._output_topic:
            if self._pub is not None:
                self.destroy_publisher(self._pub)
            self._pub = self.create_publisher(Twist, self._output_topic, 10)

        if self._sub is None or previous_input != self._input_topic:
            if self._sub is not None:
                self.destroy_subscription(self._sub)
            self._sub = self.create_subscription(
                Twist,
                self._input_topic,
                self._on_cmd_vel,
                10,
            )

        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main() -> None:
    rclpy.init()
    node = DiffDriveCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
