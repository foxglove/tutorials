#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

class Sensor: public rclcpp::Node{
private:
    void objectCallback(){
        // All transforms must be correctly timestamped
        pose_.header.stamp = this->get_clock()->now();
        pose_.header.frame_id = "sensor_link";
        if (count_%2) {
            pose_.pose.position.x = 1.0;
            pose_.pose.position.y = 1.0;
        } else {
            pose_.pose.position.x = 2.0;
            pose_.pose.position.y = 3.0;
        }
        // Change the detected object's position, depending on whether count_ is even or odd
        count_++;
        pose_pub_->publish(pose_);
    }

    // Timer for the simulated detected object
    rclcpp::TimerBase::SharedPtr object_timer_;
    // Publisher for the simulated detected object
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    // Pose to publish
    geometry_msgs::msg::PoseStamped pose_;
    // Aux variable that determines the detected object's position
    int count_ = 0;

public:
    Sensor(const std::string & name):Node(name){
        // Initialize timer for object detection
        object_timer_ = create_wall_timer(std::chrono::seconds(1), 
            std::bind(&Sensor::objectCallback, this));

        // Initialize publisher for object detection
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/detected_object",10);    
    }

    ~Sensor(){}
};


int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sensor>("sensor");
    rclcpp::spin(node);
    return 0;
}
