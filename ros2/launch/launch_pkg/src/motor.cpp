#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Declare a MotorNode
class MotorNode : public rclcpp::Node{
public:
    MotorNode(): Node("motor_node"){
        // Create a subscription for the command
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",rclcpp::SystemDefaultsQoS(), 
            std::bind(&MotorNode::cmdVelCallback, this, std::placeholders::_1)
        );
    }

    ~MotorNode(){}

private:
    // The cmdVelCallback will be executed each time we receive a new commands
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        // Prints the direction of the motors
        RCLCPP_INFO(get_logger(),"Received value %.2f", msg->linear.x);
        if(msg->linear.x>0.0){
            RCLCPP_INFO(get_logger(),"Moving motors forward");
        }else if(msg->linear.x<0.0){
            RCLCPP_INFO(get_logger(),"Moving motors backwards");
        }else{
            RCLCPP_INFO(get_logger(),"Stopping motors");
        }
    }
    // ROS Subs
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorNode>();
    rclcpp::spin(node);
    return 0;
}

