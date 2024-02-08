#include <rclcpp/rclcpp.hpp>
#include "minimal_timer/minimal_timer_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::MinimalTimer>());
    rclcpp::shutdown();
    return 0;
}