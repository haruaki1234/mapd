#include <rclcpp/rclcpp.hpp>
#include "route_subscriber/route_subscriber_component.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<tlab::RouteSubscriber>());
    rclcpp::shutdown();
    return 0;
}