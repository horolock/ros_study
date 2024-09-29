#include "rclcpp/rclcpp.hpp"

#include "mmwave_vision/srv/mm_wave_command_service.hpp"
#include "serial.h"

class MMWaveCommandNode: public rclcpp::Node
{
public:
    MMWaveCommandNode(): Node("mmwave_command") {

    }

private:
    void handle_command_request(
        const std::shared_ptr<mmwave_vision::srv::MMWaveCommandService::Request> req,
        const std::shared_ptr<mmwave_vision::srv::MMWaveCommandService::Response> res
    ) {

        serial
    }
    rclcpp::Service<mmwave_vision::srv::MMWaveCommandService>::SharedPtr service;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MMWaveCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;   
}