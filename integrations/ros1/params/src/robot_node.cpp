#include <ros/ros.h>
using namespace std::chrono_literals;

class RobotNode : ros::NodeHandle{
private:
    // First we declare the private attributes of our node
    std::string robot_name_;
    float max_speed_, last_max_speed_;
    std::vector<std::string> waypoints_;

    // Define a timer for periodic check
    ros::Timer timer_;

public:
    // Initializer of the Node
    RobotNode()
    :NodeHandle("~"){
        // Auxiliary variable
        std::string param_name;
        // Get the parameters 
        if(searchParam("robot_name",param_name)){
            getParam(param_name, robot_name_);
        }else{
            ROS_WARN("Parameter 'robot_name' not defined");
        }
        if(searchParam("max_speed",param_name)){
            getParam(param_name, max_speed_);
        }else{
            ROS_WARN("Parameter 'max_speed' not defined");
        }
        if(searchParam("waypoints",param_name)){
            getParam(param_name, waypoints_);
        }else{
            ROS_WARN("Parameter 'waypoints' not defined");
        }

        // Show the robot's attributes loaded with parameters
        ROS_INFO("Hi! I'm '%s'", robot_name_.c_str());
        ROS_INFO("My max speed is %.1f", max_speed_);
        ROS_INFO("I will follow the waypoints: ");
        for (unsigned int i = 0;i<waypoints_.size();i++){
            ROS_INFO("%d) %s", i+1, waypoints_[i].c_str());
        }

        // Create a periodic timer of 1 second
        timer_ = createTimer(ros::Duration(1), std::bind(&RobotNode::timerCallback, this));
    }

    // Destructor of the Node
    ~RobotNode(){
        ROS_INFO( "'%s' says bye!", robot_name_.c_str());
    }

    // Timer function to show the changes in max_speed
    void timerCallback(){
        // Store current max speed
        last_max_speed_ = max_speed_;
        // Get the speed parameter again to check for changes
        ros::param::get("/robot_node/max_speed", max_speed_);
        // Print new max speed
        if(max_speed_ != last_max_speed_){
           ROS_INFO("My NEW max speed is %.1f", max_speed_);
        }
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "robot_node");

    auto robot_node = RobotNode(); 

    ros::spin();
    return 0;
}
