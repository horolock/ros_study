#include "rclcpp/rclcpp.hpp"

#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisher: public rclcpp::Node
{
public:
    HardwareStatusPublisher(): Node("hardware_status_publisher") {
        m_timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HardwareStatusPublisher::publish_hardware_status, this)
        );
        m_pub = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("/hardware_status", 10);
    }

private:
    /**
     * Publishing hardware status callback function
     */
    void publish_hardware_status() {
        my_robot_interfaces::msg::HardwareStatus msg;
        msg.temperature = 57;
        msg.are_motors_ready = false;
        msg.debug_message = "Motors are too hot!";

        m_pub->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}