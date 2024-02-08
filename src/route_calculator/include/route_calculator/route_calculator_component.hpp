#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace tlab
{

class RouteCalculator : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
public:
    RouteCalculator(const rclcpp::NodeOptions& options) : RouteCalculator("", options) {}
    RouteCalculator(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : 
        Node("route_calcurator_node", name_space, options)
    {
        _subscription = this->create_subscription<std_msgs::msg::String>(
            "topic_test",
            rclcpp::QoS(10),
            [this](const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
            }
        );
    }
};

} // namespace tlab