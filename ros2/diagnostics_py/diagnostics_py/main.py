import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int16
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class DiagnosticsPublisher(Node):
    def __init__(self) -> None:
        super().__init__('diagnostics_publisher')
        self.timer_count = 0.0
        self.value = 0
        self.last_msg_time = self.get_clock().now()

        qos_profile = QoSProfile(depth=10)

        # Publisher for diagnostic array messages
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile)

        # Subscriber to the Int16 topic
        self.topic_sub = self.create_subscription(Int16, '/topic', self.topic_callback, qos_profile)

        # Timer to update status based on internal counter
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Timer to check for staleness
        self.staleness_timer = self.create_timer(0.5, self.staleness_check_callback)

        self.topic_status = DiagnosticStatus()
        self.self_status = DiagnosticStatus()

    # Callback to get the value and time for the topic
    def topic_callback(self, msg) -> None:
        self.value = msg.data
        self.last_msg_time = self.get_clock().now()
        self.update_topic_status()

    # Timer to check if the topic is STALE
    def staleness_check_callback(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_msg_time).nanoseconds > 1e9:  # 1 second
            self.topic_status.level = DiagnosticStatus.STALE
            self.topic_status.message = "No data received for more than 1 second"

    # Callback to increase the timer count and publish the diagnostic
    def timer_callback(self) -> None:
        self.timer_count += 0.1

        self.update_self_status()

        if self.timer_count >= 10.0:
            self.timer_count = 0.0

        # Publish diagnostic array
        diag_array_msg = DiagnosticArray()
        diag_array_msg.header.stamp = self.get_clock().now().to_msg()
        diag_array_msg.status.append(self.topic_status)
        diag_array_msg.status.append(self.self_status)
        self.diagnostics_pub.publish(diag_array_msg)

    # Function to update the Status for the topic
    def update_topic_status(self) -> None:
        self.topic_status.name = "Int16 Topic Status"
        self.topic_status.hardware_id = "int16_topic_001"
        self.topic_status.values.clear()
        key_value = KeyValue()
        key_value.key = "Last value"
        key_value.value = str(self.value)
        self.topic_status.values.append(key_value)

        if self.value <= 1000:
            self.topic_status.level = DiagnosticStatus.OK
            self.topic_status.message = "Value within normal range"
        elif self.value <= 1500:
            self.topic_status.level = DiagnosticStatus.WARN
            self.topic_status.message = "Value approaching upper limit"
        else:
            self.topic_status.level = DiagnosticStatus.ERROR
            self.topic_status.message = "Value exceeds upper limit"

    # Function to update the Status for the self timer
    def update_self_status(self) -> None:
        self.self_status.name = "Self Status"
        self.self_status.hardware_id = "Self_001"
        self.self_status.values.clear()

        key_value = KeyValue()
        key_value.key = "Timer count"
        key_value.value = str(self.timer_count)
        self.self_status.values.append(key_value)

        if self.timer_count <= 3:
            self.self_status.level = DiagnosticStatus.OK
            self.self_status.message = "30% completed"
        elif self.timer_count <= 6:
            self.self_status.level = DiagnosticStatus.WARN
            self.self_status.message = "60% completed"
        elif self.timer_count <= 9:
            self.self_status.level = DiagnosticStatus.ERROR
            self.self_status.message = "90% completed"

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiagnosticsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
