#include <rclcpp/rclcpp.hpp>
#include "mapd_interpolation/mapd_interpolation_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::MapdInterpolation>());
    rclcpp::shutdown();
    return 0;
}