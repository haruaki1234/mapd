#include <rclcpp/rclcpp.hpp>
#include "route_calculator/route_calculator_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::RouteCalculator>());
    rclcpp::shutdown();
    return 0;
}