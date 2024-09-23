#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberCounterNode: public rclcpp::Node 
{
public:
    NumberCounterNode(): Node("number_counter") {
        mNumberSubscription = this->create_subscription<std_msgs::msg::Int64>("/number", 10, std::bind(&NumberCounterNode::numberCallback, this, std::placeholders::_1));
        mCounterPublisher = this->create_publisher<std_msgs::msg::Int64>("/number_count", 10);
    }

private:
    void numberCallback(const std_msgs::msg::Int64::SharedPtr msg) {
        mCounterPublisher->publish(*msg);
    }

    /* Subscriptions */
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr mNumberSubscription;

    /* Publishers */
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr mCounterPublisher;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}