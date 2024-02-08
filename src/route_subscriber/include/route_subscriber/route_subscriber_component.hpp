#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


namespace tlab
{

class RouteSubscriber : public rclcpp::Node {
private:
public:
    RouteSubscriber(const rclcpp::NodeOptions& options) : RouteSubscriber("", options) {}
    RouteSubscriber(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : 
        Node("route_subscriber_node", name_space, options)
    {
        using namespace std::chrono_literals;
        // goal_poseトピックのパブリッシャーを作成し、10のQoSポリシーで定義している
        // static auto goal_pos_pub = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::QoS(10).reliable());
        
        

    }
                   
};

} // namespace tlab