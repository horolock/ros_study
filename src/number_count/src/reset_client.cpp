#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"

class ResetClientNode: public rclcpp::Node 
{
public:
    ResetClientNode(): Node("reset_client") {
        m_request_thread = std::thread(std::bind(
            &ResetClientNode::send_reset_request, this, true
        ));
    }

    void send_reset_request(bool req) {
        auto client = this->create_client<std_srvs::srv::SetBool>("reset_service");

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Failed to wait for service.");
        }

        /* Create request */
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = req;

        /* Request with future */
        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response : %s", response->message.c_str());
        } catch(std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get response.");
        }
    }

private:
    std::thread m_request_thread;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ResetClientNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}