#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class TfBroadcaster : ros::NodeHandle{
private:
    // Declare a timer for the tf_broadcaster
    ros::Timer transform_timer_;
    // Declare the transforms broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    // Subscription for cmdVel
    ros::Subscriber cmd_vel_sub_;
    // Declare the transform to broadcast
    geometry_msgs::TransformStamped transf_;


    void armCallback(const geometry_msgs::Twist msg){
        transf_.transform.translation.x = msg.linear.x;
        transf_.transform.translation.y = msg.linear.y;
        tf2::Quaternion q;
        q.setRPY(0,0,msg.angular.z);
        transf_.transform.rotation.x = q.getX();
        transf_.transform.rotation.y = q.getY();
        transf_.transform.rotation.z = q.getZ();
        transf_.transform.rotation.w = q.getW();
    }

    void tfTimer(){
        // All transforms must be correctly time-stamped!
        transf_.header.stamp = ros::Time::now();
        tf_broadcaster_.sendTransform(transf_);
    }

public:
    TfBroadcaster():
    NodeHandle("~"){
        // Initialize the transforms broadcaster
        tf_broadcaster_ = tf2_ros::TransformBroadcaster();
        // Define the "parent" frame
        transf_.header.frame_id = "arm_base_link";
        // Define the "child" frame
        transf_.child_frame_id = "arm_end_link";
        // Initialize timer for publishing transform
        transform_timer_ = createTimer(ros::Duration(1), 
            std::bind(&TfBroadcaster::tfTimer, this));
        // Create subscriber for cmdVel
        cmd_vel_sub_ = subscribe<geometry_msgs::Twist>(
            "/cmd_vel",10, &TfBroadcaster::armCallback, this);            
    }

    ~TfBroadcaster(){}
};

int main(int argc, char **argv){
    ros::init(argc, argv, "tf_broadcaster");
    auto node = TfBroadcaster();
    ros::spin();
    return 0;
}
