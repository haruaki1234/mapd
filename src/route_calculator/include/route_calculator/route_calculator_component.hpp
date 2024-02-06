#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tlab
{

class RouteCalculator : public rclcpp::Node {
private:
public:
    RouteCalculator(const rclcpp::NodeOptions& options) : RouteCalculator("", options) {}
    RouteCalculator(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : 
        Node("route_calcurator_node", name_space, options)
    {
        RCLCPP_INFO(this->get_logger(), "Hello");
    }
};

} // namespace tlab