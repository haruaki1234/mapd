#include <rclcpp/rclcpp.hpp>
#include "mapd_client/mapd_client_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::MapdClient>());
    rclcpp::shutdown();
    return 0;
}