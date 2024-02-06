#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tlab
{

class RouteController : public rclcpp::Node {
private:
public:
    RouteController(const rclcpp::NodeOptions& options) : RouteController("", options) {}
    RouteController(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : 
        Node("route_controller_node", name_space, options)
    {
        RCLCPP_INFO(this->get_logger(), "haruaki");
    }
};

} // namespace tlab