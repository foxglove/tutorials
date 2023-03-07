#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

class Sensor:  ros::NodeHandle{
private:
  // Timer for the simulated detected object
  ros::Timer object_timer_;
  // Publisher for the simulated detected object
  ros::Publisher pose_pub_;
  // Pose to publish
  geometry_msgs::PoseStamped pose_;
  // Aux variable that determines the detected object’s position
  int count_ = 0;

  void objectCallback(){
    // All transforms must be correctly time-stamped
    pose_.header.stamp = ros::Time::now();
    pose_.header.frame_id = "sensor_link";

    if (count_ % 2) {
      pose_.pose.position.x = 1.0;
      pose_.pose.position.y = 1.0;
    } else {
      pose_.pose.position.x = 2.0;
      pose_.pose.position.y = 3.0;
    }

    // Change the detected object’s position, depending on whether count_ is even or odd
    count_++;
    pose_pub_.publish(pose_);
  }

  

public:
  Sensor():
  NodeHandle("~"){
    // Initialize timer for object detection
    object_timer_ = createTimer(ros::Duration(1), 
      std::bind(&Sensor::objectCallback, this));

    // Initialize pub for object detection
    pose_pub_ = advertise<geometry_msgs::PoseStamped>(
      "/detected_object", 10);
  }

  ~Sensor(){}
};


int main(int argc, char **argv){
  ros::init(argc, argv, "sensor");
  auto node = Sensor();
  ros::spin();
  return 0;
}
