#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// Declare a ComputeNode
class ComputeNode : public ros::NodeHandle{
public:
    ComputeNode(): NodeHandle(){
        // Declare a subscription for sensor data
        sensor_sub_ = subscribe("sensor_data", 10, &ComputeNode::sensorCallback, this);

        // Declare a publisher for motors command
        cmd_vel_pub_ = advertise<geometry_msgs::Twist>("cmd_vel", 10);
    }

    ~ComputeNode(){}

private:
    // The sensorCallback will be executed each time we receive new data
    void sensorCallback(const std_msgs::Int16 msg){
        // Give a command depending on sensor data and publish
        if(msg.data<-1000){
            cmd_vel_msg_.linear.x = -1.0;
        }
        else if(msg.data>1000){
            cmd_vel_msg_.linear.x = 1.0;
        }
        else{
            cmd_vel_msg_.linear.x = 0.0;
        }
        ROS_INFO("Received value %d, sending %.2f", msg.data, cmd_vel_msg_.linear.x);
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }
    // ROS Subs
    ros::Subscriber sensor_sub_;

    // ROS Pubs
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist cmd_vel_msg_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "compute_node");
    auto node = ComputeNode();
    ros::spin();
    return 0;
}

