#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Declare a ComputeNode
class ComputeNode : public rclcpp::Node{
public:
    ComputeNode(): Node("compute_node"){
        // Declare a subscription for sensor data
        sensor_sub_ = create_subscription<std_msgs::msg::Int16>(
            "sensor_data",rclcpp::SystemDefaultsQoS(), 
            std::bind(&ComputeNode::sensorCallback, this, std::placeholders::_1)
        );
        // Declare a publisher for motors command
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            rclcpp::SystemDefaultsQoS()
        );
    }

    ~ComputeNode(){}

private:
    // The sensorCallback will be executed each time we receive new data
    void sensorCallback(const std_msgs::msg::Int16::SharedPtr msg){
        // Give a command depending on sensor data and publish
        if(msg->data<-1000){
            cmd_vel_msg_.linear.x = -1.0;
        }
        else if(msg->data>1000){
            cmd_vel_msg_.linear.x = 1.0;
        }
        else{
            cmd_vel_msg_.linear.x = 0.0;
        }
        RCLCPP_INFO(get_logger(),"Received value %d, sending %.2f", msg->data, cmd_vel_msg_.linear.x);
        cmd_vel_pub_->publish(cmd_vel_msg_);
    }
    // ROS Subs
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sensor_sub_;

    // ROS Pubs
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::Twist cmd_vel_msg_;
};

int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputeNode>();
    rclcpp::spin(node);
    return 0;
}

