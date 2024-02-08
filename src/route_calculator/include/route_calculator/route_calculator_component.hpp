#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>

namespace tlab
{

const size_t ROBOT_NUM = 3;

class RouteCalculator : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;
    rclcpp::Subscription<my_msgs::msg::RouteRequest>::SharedPtr route_request_subscription;

    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> calc_path_publisher_;

public:
    RouteCalculator(const rclcpp::NodeOptions& options) : RouteCalculator("", options) {}
    RouteCalculator(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_calcurator_node", name_space, options)
    {
        // _subscription = this->create_subscription<std_msgs::msg::String>(
        //     "topic_test",
        //     rclcpp::QoS(10),
        //     [this](const std_msgs::msg::String::SharedPtr msg){
        //     RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
        //     }
        // );

        // pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "pose_topic_test",
        //     rclcpp::QoS(10),
        //     [&](const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg){
        //     RCLCPP_INFO(this->get_logger(), "I heard: %lf", pose_msg->pose.position.x);
        //     }
        // );

        // path_subscription = this->create_subscription<nav_msgs::msg::Path>("path_topic_test", rclcpp::QoS(10), [&](const nav_msgs::msg::Path::SharedPtr path_msg) {
        //     RCLCPP_INFO(this->get_logger(), "I heard: %lf", path_msg->poses.back().pose.position.x);
        //     // RCLCPP_INFO(this->get_logger(), "I heard: %lf", path_msg->poses[0].pose.position.x);
        // });

        for (size_t i = 0; i < ROBOT_NUM; i++) {
            calc_path_publisher_.push_back(this->create_publisher<nav_msgs::msg::Path>("calc_path_" + std::to_string(i), rclcpp::QoS(10))); // topic name : "calc_path_0" , "calc_path_1" , ...
        }

        route_request_subscription = this->create_subscription<my_msgs::msg::RouteRequest>("route_request_topic_test", rclcpp::QoS(10), [&](const my_msgs::msg::RouteRequest::SharedPtr route_request_msg) {
            RCLCPP_INFO(this->get_logger(), "I heard: %lf", route_request_msg->goal.pose.position.x);

            auto route_calc_msg = std::make_shared<nav_msgs::msg::Path>();
            route_calc_msg->poses.push_back(route_request_msg->start);
            route_calc_msg->poses.push_back(route_request_msg->goal);
            calc_path_publisher_[route_request_msg->robot_no]->publish(*route_calc_msg);
            // RCLCPP_INFO(this->get_logger(), "I heard: %lf", path_msg->poses[0].pose.position.x);
        });
    }
};

} // namespace tlab