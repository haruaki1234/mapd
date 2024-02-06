#include <rclcpp/rclcpp.hpp>
#include "route_controller/route_controller_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::RouteController>());
    rclcpp::shutdown();
    return 0;
}