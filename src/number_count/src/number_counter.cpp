#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_srvs/srv/set_bool.hpp"


class NumberCounterNode: public rclcpp::Node 
{
public:
    NumberCounterNode(): Node("number_counter") {
        m_counter = 0;
        mNumberSubscription = this->create_subscription<std_msgs::msg::Int64>("/number", 10, std::bind(&NumberCounterNode::numberCallback, this, std::placeholders::_1));
        mCounterPublisher = this->create_publisher<std_msgs::msg::Int64>("/number_count", 10);
        
        mResetService = this->create_service<std_srvs::srv::SetBool>(
            "reset_service",
             std::bind(&NumberCounterNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void numberCallback(const std_msgs::msg::Int64::SharedPtr msg) {
        (void)msg;
        std_msgs::msg::Int64 counter_msg;
        counter_msg.data = m_counter++;
        mCounterPublisher->publish(counter_msg);
    }

    void resetCallback(
        const std_srvs::srv::SetBool::Request::SharedPtr req, 
        const std_srvs::srv::SetBool::Response::SharedPtr res) {
            if (req->data) {
                RCLCPP_INFO(this->get_logger(), "Reset request received.");
                m_counter = 0;
                res->message = "Successfully set 0.";
            }
        }

    /* Subscriptions */
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr mNumberSubscription;

    /* Publishers */
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr mCounterPublisher;

    /* Services */
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mResetService;

    int64_t m_counter;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}