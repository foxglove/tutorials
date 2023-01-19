// Standard C++ API for interacting with ROS 2
#include <rclcpp/rclcpp.hpp>
// Useful namespace for durations
using namespace std::chrono_literals;

class RobotNode : public rclcpp::Node{
private:
    // Declare private attributes of our node
    std::string robot_name_;
    float max_speed_, last_max_speed_;
    std::vector<std::string> waypoints_;

    // Define periodic timer
    rclcpp::TimerBase::SharedPtr timer_;

public:
    // Initialize class
    RobotNode(const std::string & name)
    :Node(name){
        // Declare parameters
        declare_parameter("robot_name", "");
        declare_parameter("max_speed", 0.0);
        declare_parameter("waypoints");

        // Read parameters
        get_parameter("robot_name", robot_name_);
        get_parameter("max_speed", max_speed_);
        get_parameter("waypoints", waypoints_);

        // Show robot's attributes loaded with parameters
        RCLCPP_INFO(get_logger(),"Hi! I'm '%s'", robot_name_.c_str());
        RCLCPP_INFO(get_logger(),"My max speed is %.1f", max_speed_);
        RCLCPP_INFO(get_logger(),"I will follow the waypoints: ");
        for (unsigned int i = 0; i < waypoints_.size(); i++) {
            RCLCPP_INFO(get_logger(),"%d) %s", i+1, waypoints_[i].c_str());
        }

        // Create periodic 1s timer
        timer_ = create_wall_timer(1s, std::bind(&RobotNode::timerCallback, this));
    }

    // Destructor
    ~RobotNode(){
        RCLCPP_INFO(get_logger(), "'%s' says bye!", robot_name_.c_str());
    }

    // Show changes in max_speed
    void timerCallback(){
        // Store current max speed
        last_max_speed_ = max_speed_;
        // Check speed parameter again for changes
        get_parameter("max_speed", max_speed_);
        // Print new max speed
        if (max_speed_ != last_max_speed_) {
           RCLCPP_INFO(get_logger(),"My NEW max speed is %.1f", max_speed_);
        }
    }
};


int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto robot_node = std::make_shared<RobotNode>("robot_node");
    rclcpp::spin(robot_node);
    return 0;
}
