#include <rclcpp/rclcpp.hpp>
#include "mapd_followup/mapd_followup_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::MapdFollowup>());
    rclcpp::shutdown();
    return 0;
}