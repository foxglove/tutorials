import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_ros2_package.action import Countdown

class CountdownClient(Node):
    def __init__(self):
        # Initializes "countdown_client" node
        super().__init__("countdown_client")
        self._action_client = ActionClient(self, Countdown, "countdown")
        
    # Waits for server to be available, then sends goal
    def send_goal(self, starting_num):
        goal_msg = Countdown.Goal()
        goal_msg.starting_num = starting_num
        self._action_client.wait_for_server()

        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Register a callback for when future is complete (i.e. server accepts or rejects goal request)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Run when client sends goal
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_num))

    def goal_response_callback(self, future):
        # Get handle for the goal we just sent
        goal_handle = future.result()

        # Return early if goal is rejected
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Use goal handle to request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        # Log result and shut down ROS 2 cleanly
        self.get_logger().info('Result: {0}'.format(result.is_finished))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = CountdownClient()

    # Sends goal and waits until it’s completed
    future = action_client.send_goal(10)
    rclpy.spin(action_client, future)

if __name__ == '__main__':
    main()
