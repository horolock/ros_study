#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsServerNode: public rclcpp::Node {
public:
    AddTwoIntsServerNode(): Node("add_two_ints_server") {
        /* Initialize server */
        m_server = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", 
            std::bind(
                &AddTwoIntsServerNode::AddTwoIntsCallback, 
                this, 
                std::placeholders::_1, std::placeholders::_2
            )
        );

        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }
private:
    void AddTwoIntsCallback(
        const example_interfaces::srv::AddTwoInts::Request::SharedPtr req, 
        const example_interfaces::srv::AddTwoInts::Response::SharedPtr res) {
            res->sum = req->a + req->b;
            RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", req->a, req->b, res->sum);
        }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr m_server;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}