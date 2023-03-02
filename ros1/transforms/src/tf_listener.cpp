#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class TfListener : ros::NodeHandle{
private:
  // Subscription to pose published by sensor node
  ros::Subscriber pose_sub_;
  // Buffer that stores several seconds of transforms for easy lookup by the listener
  tf2_ros::Buffer tf_buffer_;
  // Listener for the broadcast transform message
  tf2_ros::TransformListener* tf_listener_;
  // Pose in source frame (`sensor_link`)
  geometry_msgs::PoseStamped pose_in_;
  // Pose in target frame (`arm_end_link`)
  geometry_msgs::PoseStamped pose_out_;
  
  void poseCallback(const geometry_msgs::PoseStamped msg) {
    try {
      // Store the incoming pose in pose_in_
      pose_in_ = msg;
      // Transforms the pose between the source frame and target frame
      tf_buffer_.transform<geometry_msgs::PoseStamped>(pose_in_, pose_out_, 
        "arm_end_link", ros::Duration(1));
      // Log coordinates of pose in target frame
      ROS_INFO("Object pose in 'arm_end_link' is:\n x,y,z = %.1f,%.1f,%.1f",
        pose_out_.pose.position.x,
        pose_out_.pose.position.y,
        pose_out_.pose.position.z);
    } catch (const tf2::TransformException & ex) {
      ROS_WARN("Could not find object position in 'arm_end_link' frame.");
      return;
    }
  }
 

public:
  TfListener():
  NodeHandle("~") {   
    // Listen to the buffer of transforms
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    // Subscribe to pose published by sensor node (check every second)
    pose_sub_ = subscribe<geometry_msgs::PoseStamped>
      ("/detected_object", 10, &TfListener::poseCallback, this);
  }
  
  ~TfListener(){}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_listener");
  TfListener node;
  ros::spin();
  return 0;
}
