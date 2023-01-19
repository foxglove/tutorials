#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class TfBroadcaster : public rclcpp::Node{
private:
    void armCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        transf_.transform.translation.x = msg->linear.x;
        transf_.transform.translation.y = msg->linear.y;
        tf2::Quaternion q;
        q.setRPY(0,0,msg->angular.z);
        transf_.transform.rotation.x = q.getX();
        transf_.transform.rotation.y = q.getY();
        transf_.transform.rotation.z = q.getZ();
        transf_.transform.rotation.w = q.getW();
    }


    void tfTimer(){
        // All transforms must be correctly time-stamped!
        transf_.header.stamp = this->get_clock()->now();
        tf2_broadcaster_->sendTransform(transf_);
    }

    // Declare a timer for the tf2_broadcaster
    rclcpp::TimerBase::SharedPtr transform_timer_;
    // Declare the transforms broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
    // Subscription for cmdVel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    // Declare the transform to broadcast
    geometry_msgs::msg::TransformStamped transf_;
    

public:
    TfBroadcaster(const std::string & name):
    Node(name){
        // Initialize the transforms broadcaster
        tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Define the "parent" frame
        transf_.header.frame_id = "base_link";
        // Define the "child" frame
        transf_.child_frame_id = "arm_link";
        // Initialize timer for publishing transform
        transform_timer_ = create_wall_timer(std::chrono::seconds(1), 
            std::bind(&TfBroadcaster::tfTimer, this));
        // Create subscriber for cmdVel
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",10,std::bind(&TfBroadcaster::armCallback, this, std::placeholders::_1));            
    }

    ~TfBroadcaster(){}
};

int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfBroadcaster>("tf_broadcaster");
    rclcpp::spin(node);
    return 0;
}
