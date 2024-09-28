/**
 * Service need to use future features. Not a `spin`
 */

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode: public rclcpp::Node
{
public:
    AddTwoIntsClientNode(): Node("add_two_ints_client") {
        m_thread1 = std::thread(std::bind(
            &AddTwoIntsClientNode::call_add_two_ints_service, this, 1,  4
        ));
    }

    void call_add_two_ints_service(int a, int b) {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        /**
         * Waiting for server is up.
         */
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up ...");
        }

        /* Create request */
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        /* Create future object */
        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result : %d + %d = %ld", a, b, response->sum);
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Future error occured: %s", e.what());
        }
    }

private:
    std::thread m_thread1;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}