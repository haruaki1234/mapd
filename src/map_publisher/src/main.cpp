#include <rclcpp/rclcpp.hpp>
#include "map_publisher/map_publisher_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::MapPublisher>());
    rclcpp::shutdown();
    return 0;
}