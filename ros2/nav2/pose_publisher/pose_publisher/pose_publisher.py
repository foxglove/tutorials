import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Create a publisher for PoseStamped messages
        self.publisher_129_ = self.create_publisher(
            PoseStamped, 'pose_129', 10)
        self.publisher_135_ = self.create_publisher(
            PoseStamped, 'pose_135', 10)
        self.publisher_139_ = self.create_publisher(
            PoseStamped, 'pose_139', 10)

        # Create a TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to publish at regular intervals
        self.timer_129 = self.create_timer(
            0.5, lambda: self.publish_pose("bifrustrum_08_cam129"))
        self.timer_135 = self.create_timer(
            0.5, lambda: self.publish_pose("bifrustrum_08_cam135"))
        self.timer_130 = self.create_timer(
            0.5, lambda: self.publish_pose("bifrustrum_08_cam139"))

    def publish_pose(self, frame):
        try:
            # Lookup transform (replace 'target_frame' and 'source_frame' with your frames)
            trans = self.tf_buffer.lookup_transform(
                frame, 'world_link', rclpy.time.Time())

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world_link'
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation

            # Publish the pose
            self.publisher_129_.publish(pose_msg)
            self.get_logger().info('Published Pose: [%f, %f, %f]' % (
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z))

        except Exception:
            self.get_logger().warn('Could not get transform')


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
