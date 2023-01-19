#include <ros/ros.h>
#include <std_msgs/Int16.h>

// Declare a SensorNode
class SensorNode : public ros::NodeHandle{
public:
    SensorNode(): NodeHandle(){
        // Create a timer to read the sensor each second
        timer_ = createTimer(ros::Duration(1), std::bind(&SensorNode::timerCallback, this));

        // Create a publisher for the sensor data
        sensor_pub_ = advertise<std_msgs::Int16>("sensor_data", 10);
    }

    ~SensorNode(){}

private:
    // The timerCallback is executed each second. 
    void timerCallback(){
        // Simulate sensor reading and publish
        sensor_msg_.data = int16_t(rand());
        ROS_INFO("Read value %d", sensor_msg_.data);
        sensor_pub_.publish(sensor_msg_);
    }
    // ROS Timer
    ros::Timer timer_;
    // ROS Pubs
    ros::Publisher sensor_pub_;
    std_msgs::Int16 sensor_msg_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "sensor_node");
    auto node = SensorNode();
    ros::spin();
    return 0;
}

