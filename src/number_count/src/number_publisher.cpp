#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPublisherNode: public rclcpp::Node {
public:
    NumberPublisherNode(): Node("number_publisher"), mCounter(0) {
        mTimer = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberPublisherNode::timerCallback, this)
        );

        mNumberPublisher = this->create_publisher<std_msgs::msg::Int64>("/number", 10);

        RCLCPP_INFO(this->get_logger(), "Number publisher node created.");
    }

private:
    /* Callbacks */
    void timerCallback() {
        std_msgs::msg::Int64 msg;
        msg.set__data(mCounter);

        RCLCPP_INFO(this->get_logger(), "Number %ld", mCounter);

        mNumberPublisher->publish(msg);

        mCounter++;
    }

    /* Publisher */
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr mNumberPublisher;
    
    /* Member variables */
    int64_t mCounter;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}