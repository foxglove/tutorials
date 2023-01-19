import rospy
import actionlib
import time
import my_ros1_package.msg

class CountdownServer():
    # Create Feedback and Result messages
    def __init__(self):
        # Create the server
        self._action_server = actionlib.SimpleActionServer('countdown', my_ros1_package.msg.CountdownAction, self.execute_callback, False)

        # Start the server
        self._action_server.start()
        rospy.loginfo("Starting Action Server")
	
    # Callback function to run after acknowledging a goal from the client
    def execute_callback(self, goal_handle):
        rospy.loginfo("Starting countdown…")


        result = my_ros1_package.msg.CountdownResult()
        result.is_finished = True
        # Indicate that the goal was successful
        self._action_server.set_succeeded(result)
	
def main(args=None):
   # Init ROS1 and give the node a name
   rospy.init_node("countdown_server")
   countdown_server = CountdownServer()
   rospy.spin()

if __name__ == '__main__':
   main()
