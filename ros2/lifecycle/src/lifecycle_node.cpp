#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <lifecycle_msgs/msg/transition.hpp>

using LifecycleReturn=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleNodeExample: public rclcpp_lifecycle::LifecycleNode{
public:
// Instantiation of the node
LifecycleNodeExample(const std::string & name="lifecycle_node"):
LifecycleNode(name){

}

~LifecycleNodeExample(){}

private:
// Function is called when transitioning to Configured State
LifecycleReturn on_configure(const rclcpp_lifecycle::State & state){
  RCLCPP_INFO(get_logger(),"Node '%s' is in state '%s'. Transitioning to 'configure'", get_name(), state.label().c_str());

  // If all is good, transition is successful
  return LifecycleReturn::SUCCESS;
}

// Function called when transtitionig to Active State
LifecycleReturn on_activate(const rclcpp_lifecycle::State & state){
  RCLCPP_INFO(get_logger(),"Node '%s' is in state '%s'. Transitioning to 'activate'", get_name(), state.label().c_str());

  return LifecycleReturn::SUCCESS;
}

LifecycleReturn on_deactivate(const rclcpp_lifecycle::State & state){
  RCLCPP_INFO(get_logger(),"Node '%s' is in state '%s'. Transitioning to 'deactivate'", get_name(), state.label().c_str());

  return LifecycleReturn::SUCCESS;
}

LifecycleReturn on_shutdown(const rclcpp_lifecycle::State & state){
  RCLCPP_INFO(get_logger(),"Node '%s' is in state '%s'. Transitioning to 'shutdown'", get_name(), state.label().c_str());

  return LifecycleReturn::SUCCESS;
}

};



int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto lifecycle_node = std::make_shared<LifecycleNodeExample>("lifecycle_node");
  rclcpp::spin(lifecycle_node->get_node_base_interface());
  return 0;
}
