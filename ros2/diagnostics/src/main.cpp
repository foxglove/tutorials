#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class DiagnosticsPublisher : public rclcpp::Node
{
public:
    DiagnosticsPublisher() : 
    Node("diagnostics_publisher"), 
    timer_count_(0), 
    last_msg_time_(now())
    {
        // Publisher for diagnostic array messages
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        // Subscriber to the Int16 topic
        topic_sub_ = create_subscription<std_msgs::msg::Int16>(
            "/topic", 10, std::bind(&DiagnosticsPublisher::topic_callback, this, _1));

        // Timer to update status based on internal counter
        timer_ = create_wall_timer(100ms, std::bind(&DiagnosticsPublisher::timer_callback, this));

        // Timer to check for staleness
        staleness_timer_ = create_wall_timer(500ms, std::bind(&DiagnosticsPublisher::staleness_check_callback, this));
    }

private:
    // Callback to get the value and time for the topic
    void topic_callback(const std_msgs::msg::Int16::SharedPtr msg){
        value_ = msg->data;
        last_msg_time_ = now();
        update_topic_status();
    }

    // Timer to check if the topic is STALE
    void staleness_check_callback(){
        auto now = get_clock()->now();
        if ((now - last_msg_time_).seconds() > 1.0)
        {
            topic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
            topic_status_.message = "No data received for more than 1 second";
        }
    }

    // Callback to increase the timer count and publish the diagnostic
    void timer_callback(){
        timer_count_ += 0.1;

        update_self_status();

        if (timer_count_ >= 10.0){
            timer_count_ = 0.0;
        }

        // Publish diagnostic array
        diagnostic_msgs::msg::DiagnosticArray diag_array_msg;
        diag_array_msg.header.stamp = now();
        diag_array_msg.status.push_back(topic_status_);
        diag_array_msg.status.push_back(self_status);
        diagnostics_pub_->publish(diag_array_msg);
    }
    
    // Function to update the Status for the topic 
    void update_topic_status(){
        topic_status_.name = "Int16 Topic Status";
        topic_status_.hardware_id = "int16_topic_001";
        topic_status_.values.clear();
        diagnostic_msgs::msg::KeyValue key_value;
        key_value.key = "Last value";
        key_value.value = std::to_string(value_);
        topic_status_.values.push_back(key_value);

        if (value_ <= 1000)
        {
            topic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            topic_status_.message = "Value within normal range";
        }
        else if (value_ <= 1500)
        {
            topic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            topic_status_.message = "Value approaching upper limit";
        }
        else
        {
            topic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            topic_status_.message = "Value exceeds upper limit";
        }
    }

    // Function to update the Status for the self timer
    void update_self_status(){
        self_status.name = "Self Status";
        self_status.hardware_id = "Self_001";
        self_status.values.clear();
       
        diagnostic_msgs::msg::KeyValue key_value;
        key_value.key = "Timer count";
        key_value.value = std::to_string(timer_count_);
        self_status.values.push_back(key_value);

        if(timer_count_<=3){
            self_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            self_status.message = "30% completed";
        }
        else if(timer_count_<=6){
            self_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            self_status.message = "60% completed";
        }
        else if(timer_count_<=9){
            self_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            self_status.message = "90% completed";
        }

    }

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr topic_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr staleness_timer_;

    diagnostic_msgs::msg::DiagnosticStatus topic_status_;
    diagnostic_msgs::msg::DiagnosticStatus self_status;

    double timer_count_;
    int16_t value_;
    rclcpp::Time last_msg_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiagnosticsPublisher>());
    rclcpp::shutdown();
    return 0;
}
