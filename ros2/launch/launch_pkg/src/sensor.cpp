#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

// Declare a SensorNode
class SensorNode : public rclcpp::Node{
public:
    SensorNode(): Node("sensor_node"){
        // Create a timer to read the sensor each second
        timer_ = create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&SensorNode::timerCallback, this)
        );
        // Create a publisher for the sensor data
        sensor_pub_ = create_publisher<std_msgs::msg::Int16>(
            "sensor_data", 
            rclcpp::SystemDefaultsQoS()
        );
    }

    ~SensorNode(){}

private:
    // The timerCallback is executed each second. 
    void timerCallback(){
        // Simulate sensor reading and publish
        sensor_msg_.data = int16_t(rand());
        RCLCPP_INFO(get_logger(),"Read value %d", sensor_msg_.data);
        sensor_pub_->publish(sensor_msg_);
    }
    // ROS Timer
    rclcpp::TimerBase::SharedPtr timer_;
    // ROS Pubs
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr sensor_pub_;
    std_msgs::msg::Int16 sensor_msg_;
};

int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorNode>();
    rclcpp::spin(node);
    return 0;
}

