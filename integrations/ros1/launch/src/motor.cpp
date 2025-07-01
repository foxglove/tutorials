#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


// Declare a MotorNode
class MotorNode : public ros::NodeHandle{
public:
    MotorNode(): NodeHandle(){
        // Create a subscription for the command
        cmd_vel_sub_ = subscribe("cmd_vel", 10, &MotorNode::cmdVelCallback, this);
    }

    ~MotorNode(){}

private:
    // The cmdVelCallback will be executed each time we receive a new commands
    void cmdVelCallback(const geometry_msgs::Twist msg){
        // Prints the direction of the motors
        ROS_INFO("Received value %.2f", msg.linear.x);
        if(msg.linear.x>0.0){
            ROS_INFO("Moving motors forward");
        }else if(msg.linear.x<0.0){
            ROS_INFO("Moving motors backwards");
        }else{
            ROS_INFO("Stopping motors");
        }
    }
    // ROS Subs
    ros::Subscriber cmd_vel_sub_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "motor_node");
    auto node = MotorNode();
    ros::spin();
    return 0;
}

