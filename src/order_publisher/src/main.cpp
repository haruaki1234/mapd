#include <rclcpp/rclcpp.hpp>
#include "order_publisher/order_publisher_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::OrderPublisher>());
    rclcpp::shutdown();
    return 0;
}