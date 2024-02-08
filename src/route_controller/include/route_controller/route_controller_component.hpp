#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>

namespace tlab
{

class RouteController : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
public:
    RouteController(const rclcpp::NodeOptions& options) : RouteController("", options) {}
    RouteController(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : 
        Node("route_controller_node", name_space, options)
    {
        using namespace std::chrono_literals;
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_test",rclcpp::QoS(10));
        timer_ = this->create_wall_timer(
            500ms,
            [&](){
                auto msg = std::make_shared<std_msgs::msg::String>();
                msg->data = "Hello " + std::to_string(count_++);
                RCLCPP_INFO(this->get_logger(), "Pub:%s", msg->data.c_str());
                publisher_->publish(*msg);
            }
        );
    }
};

} // namespace tlab