#include "rclcpp/rclcpp.hpp"
#include "lab1_pkg/cpp_header.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello cpp");
    }
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

