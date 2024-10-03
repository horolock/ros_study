#include "rclcpp/rclcpp.hpp"

#include "my_robot_interfaces/srv/led_command.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"

class LEDPanelNode: public rclcpp::Node
{
public:
    LEDPanelNode(): Node("led_panel") {
        m_publisher = this->create_publisher<my_robot_interfaces::msg::LEDPanelState>("/led_panel_state", 10);
        m_service = this->create_service<my_robot_interfaces::srv::LEDCommand>(
            "led_command", 
            std::bind(&LEDPanelNode::led_command_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void led_command_callback(
        const my_robot_interfaces::srv::LEDCommand::Request::SharedPtr req, 
        const my_robot_interfaces::srv::LEDCommand::Response::SharedPtr res) {
            RCLCPP_INFO(this->get_logger(), "Get request.");
            
            uint8_t requested_led = req->led_number;
            my_robot_interfaces::msg::LEDPanelState msg;

            if (requested_led > 3 || requested_led < 1) {
                RCLCPP_INFO(this->get_logger(), "You requested invalid led index");
                res->success = false;
                return;
            }

            led_panel_state[requested_led] = req->state;
            msg.first_led = led_panel_state[0];
            msg.second_led = led_panel_state[1];
            msg.third_led = led_panel_state[2];
            res->success = true;

            m_publisher->publish(msg);
        }
    
    bool led_panel_state[3] = {false, false, false};
    rclcpp::Publisher<my_robot_interfaces::msg::LEDPanelState>::SharedPtr m_publisher;
    rclcpp::Service<my_robot_interfaces::srv::LEDCommand>::SharedPtr m_service;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LEDPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}