from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):
    def __init__(self):
        super().__init__('firefly_state_publisher')

        self.declare_parameter("spin_rate", 2.0)
        self.spin_rate = self.get_parameter("spin_rate").get_parameter_value().double_value

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.loop_rate = self.create_rate(30)

        # message declarations
        self.base_trans = TransformStamped()
        self.base_trans.header.frame_id = 'base_link'
        self.base_trans.child_frame_id = 'firefly/base_link'
        self.joint_state = JointState()

        # Firefly rotor joint names (6 rotors)
        self.rotor_joint_names = [
            'firefly/rotor_0_joint',  # front_left
            'firefly/rotor_1_joint',  # left
            'firefly/rotor_2_joint',  # back_left
            'firefly/rotor_3_joint',  # back_right
            'firefly/rotor_4_joint',  # right
            'firefly/rotor_5_joint'   # front_right
        ]

        self.angle = 0.0
        self.increment = (360.0*self.spin_rate)/30.0

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = self.rotor_joint_names

                # Keep the position updates if you want, but they won't affect the rotation
                self.angle = ((self.angle + self.increment) % 360.0) - 180.0
                self.joint_state.position = [self.angle, -self.angle, self.angle, -self.angle, self.angle, -self.angle]
                
                # Set a constant rotational velocity (radians/sec)
                self.joint_state.velocity = [self.spin_rate] * len(self.rotor_joint_names)  # Try different values to adjust speed
                # joint_state.effort = [2.0] * len(rotor_joint_names)

                # send the joint state and transform
                self.joint_pub.publish(self.joint_state)
                self.broadcaster.sendTransform(self.base_trans)

                # This will adjust as needed per iteration
                self.loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    rclpy.init()
    node = StatePublisher()
    
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error in state publisher: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()