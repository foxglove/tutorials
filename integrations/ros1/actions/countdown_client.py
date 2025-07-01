import rospy
import actionlib
import my_ros1_package.msg

class CountdownClient():
    def __init__(self):
       # Initializes "countdown_client" node
       self._action_client = actionlib.SimpleActionClient("countdown", my_ros1_package.msg.CountdownAction)

    # Waits for server to be available, then sends goal
    def send_goal(self, starting_num):
        goal_msg = my_ros1_package.msg.CountdownGoal()
        goal_msg.starting_num = starting_num
        rospy.loginfo('Starting at: {0}'.format(starting_num))
        rospy.loginfo('Waiting for server...')

        self._action_client.wait_for_server()

        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self._send_goal_future = self._action_client.send_goal(goal_msg, active_cb=self.goal_response_callback, feedback_cb=self.feedback_callback, done_cb = self.get_result_callback)

        rospy.loginfo("Goal sent!")
        
    # Run when client accepts goal
    def goal_response_callback(self):
        rospy.loginfo('Goal accepted :)')

   # Run when client sends feedback
    def feedback_callback(self, feedback_msg):
        rospy.loginfo('Received feedback: {0}'.format(feedback_msg.current_num))

   # Run when client sends final result
    def get_result_callback(self, state, result):
        # Show log and exit node
        rospy.loginfo('Result: {0}'.format(result.is_finished))
        rospy.signal_shutdown("Shutting-down client node")
        
def main(args=None):
   # Init ROS1 and give the node a name
   rospy.init_node("countdown_client")
   action_client = CountdownClient()

   # Sends goal and waits until it's completed
   action_client.send_goal(10)
   rospy.spin()

if __name__ == '__main__':
   main()
