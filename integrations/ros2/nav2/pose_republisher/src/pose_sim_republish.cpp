#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class PoseRepublisher : public rclcpp::Node {
public:
  PoseRepublisher() : Node("pose_republisher") {
    // Subscribe to PoseStamped
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "input_pose", 10,
        std::bind(&PoseRepublisher::poseCallback, this, std::placeholders::_1));

    // Publisher for the modified PoseStamped
    pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  }

private:
  // Callback for PoseStamped messages
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto modified_pose = *msg;
    modified_pose.header.stamp = get_clock()->now();
    pose_publisher_->publish(modified_pose);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseRepublisher>());
  rclcpp::shutdown();
  return 0;
}
