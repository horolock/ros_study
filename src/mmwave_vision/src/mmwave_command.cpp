#include "rclcpp/rclcpp.hpp"

#include "mmwave_vision/srv/mm_wave_command_service.hpp"
#include "ros_serial/include/serial_interface.hpp"

class MMWaveCommandNode: public rclcpp::Node
{
public:
    MMWaveCommandNode(): Node("mmwave_command") {
        this->declare_parameter("command_port", rclcpp::PARAMETER_STRING);
        this->declare_parameter("command_rate", rclcpp::PARAMETER_STRING);
    }

private:
    void handle_command_request(
        const std::shared_ptr<mmwave_vision::srv::MMWaveCommandService::Request> req,
        const std::shared_ptr<mmwave_vision::srv::MMWaveCommandService::Response> res
    ) {
        std::string serial_port = this->get_parameter("command_port").as_string();
        std::string baudrate = this->get_parameter("command_rate").as_string();
        std::string start_command = "sensorStart\n";

        serial::Serial serial_object("", std::stoi(baudrate), serial::Timeout::simpleTimeout(1000));
        serial_object.setPort(serial_port.c_str());

        try {
            serial_object.open();
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "\n\nFailed to open serial port: %s\n\n", e.what());
            rclcpp::shutdown();
            exit(0);
        }

        while (serial_object.available() > 0) {
            serial_object.readline(res->resp, 1024, ":/>");
            res->resp = "";
        }

        RCLCPP_INFO(this->get_logger(), "Sending command to sensor: '%s'", req->comm.c_str());

        req->comm.append("\n");

        int bytes_sent = serial_object.write(req->comm.c_str());

        /* Read output from mmw demo*/
        serial_object.readline(res->resp, 1024, ":/>");
        RCLCPP_INFO(this->get_logger(), "Received response from sensor: '%s'", res->resp.c_str());
        serial_object.close();

        if (!start_command.compare(req->comm)) {
            rclcpp::shutdown();
            exit(0);
        }
    }

    rclcpp::Service<mmwave_vision::srv::MMWaveCommandService>::SharedPtr service;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MMWaveCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    exit(0);
    return 0;   
}