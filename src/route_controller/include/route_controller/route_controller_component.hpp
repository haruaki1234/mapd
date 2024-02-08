#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>

namespace tlab
{

class RouteController : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<my_msgs::msg::RouteRequest>::SharedPtr route_request_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    double x_value_ = 0.0;

    std::vector<nav_msgs::msg::Path> all_robot_path;

public:
    RouteController(const rclcpp::NodeOptions& options) : RouteController("", options) {}
    RouteController(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_controller_node", name_space, options), all_robot_path(3)
    {
        using namespace std::chrono_literals;
        // publisher_ =
        // this->create_publisher<std_msgs::msg::String>("topic_test",rclcpp::QoS(10));
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_topic_test", rclcpp::QoS(10));
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_topic_test", rclcpp::QoS(10));
        route_request_publisher_ = this->create_publisher<my_msgs::msg::RouteRequest>("route_request_topic_test", rclcpp::QoS(10));
        static auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.frame_id = "map"; // 基準座標を決めている、送るときだけ

        timer_ = this->create_wall_timer(500ms, [&]() {
            // auto msg = std::make_shared<std_msgs::msg::String>();
            // msg->data = "Hello " + std::to_string(count_++);
            // RCLCPP_INFO(this->get_logger(), "Pub:%s", msg->data.c_str());
            // publisher_->publish(*msg);

            // auto pose_msg =
            // std::make_shared<geometry_msgs::msg::PoseStamped>();
            // pose_msg->header.frame_id = "map";
            // pose_msg->pose.position.x =11.1;
            // RCLCPP_INFO(this->get_logger(), "Pub:%lf",
            // pose_msg->pose.position.x); pose_publisher_->publish(*pose_msg);

            // geometry_msgs::msg::PoseStamped p;
            // p.pose.position.x = 20.55 + x_value_;
            // x_value_ += 1.0;
            // path_msg->poses.push_back(p);

            // RCLCPP_INFO(this->get_logger(), "Pub:%lf", path_msg->poses.back().pose.position.x);
            // path_publisher_->publish(*path_msg);

            int send_no = 0;

            auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
            route_request_msg->robot_name = "r" + std::to_string(send_no);
            route_request_msg->start.pose.position.x = 2.5;
            route_request_msg->goal.pose.position.x = 5.5;

            for (int i = 0; i < all_robot_path.size(); i++) {
                if (send_no != i) {
                    route_request_msg->other_path.push_back(all_robot_path[i]);
                }
            }

            RCLCPP_INFO(this->get_logger(), "Pub:%s", route_request_msg->robot_name.c_str());
            RCLCPP_INFO(this->get_logger(), "Pub:%lf", route_request_msg->start.pose.position.x);
            RCLCPP_INFO(this->get_logger(), "Pub:%lf", route_request_msg->goal.pose.position.x);
            route_request_publisher_->publish(*route_request_msg);
        });
    }
};

} // namespace tlab