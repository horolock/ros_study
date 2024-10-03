#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/led_command.hpp"

class BatteryNode: public rclcpp::Node 
{
public:
    BatteryNode(): Node("battery_node") {
        m_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryNode::timer_callback, this));
    }

private:
    /**
     * After 4 seconds, battery low
     * After 6 seconds, battery full.
     */
    void timer_callback() {
        if (m_counter == 4) {
            /* Battery low */
            RCLCPP_INFO(this->get_logger(), "Battery LOW.");
            m_request_thread = std::thread(std::bind(&BatteryNode::send_request, this, 3, true));
        } else if (m_counter == 10) {
            /* Battery charged. */
            m_on_thread = std::thread(std::bind(&BatteryNode::send_request, this, 3, false));
            m_counter = 0;
            RCLCPP_INFO(this->get_logger(), "Battery FULL.");
            return;
        }
        
        m_counter++;
    }

    void send_request(uint8_t led_number, bool led_state) {
        auto client = this->create_client<my_robot_interfaces::srv::LEDCommand>("led_command");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not ready.");
        }

        /* Create request */
        auto request = std::make_shared<my_robot_interfaces::srv::LEDCommand::Request>();
        request->led_number = led_number;
        request->state = led_state;

        /* Request with future */
        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response: %s", response->success ? "Success" : "Failed");
        } catch(std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get response");
        }
    }

    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<std::thread> m_thread_pool;
    std::thread m_request_thread;
    std::thread m_on_thread;
    int8_t m_counter = 0;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}