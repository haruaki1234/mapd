#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mapd_client/mapf_client/mapf_client.hpp>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sstream>

namespace tlab
{
const int height = 15;
const int width = 25;

class MapdClient : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    // rclcpp::TimerBase::SharedPtr timer_;

public:
    MapdClient(const rclcpp::NodeOptions& options) : MapdClient("", options) {}
    MapdClient(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("mapd_client_node", name_space, options)
    {
        using namespace std::chrono_literals;
        auto calc_msg = std::make_shared<nav_msgs::msg::Path>();
        calc_msg->header.frame_id = "map2"; // 基準座標を決めている、送るときだけ
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_test", rclcpp::QoS(10));
        uint32_t id = 1;
        std::cout << "hello" << std::endl;
        MapfClient client("133.15.98.68", 9000); // サーバーのipアドレス133.15.98.24,,,,192.168.10.108
        auto result = client.request(id);

        std::cout << "id: " << result.id << std::endl;
        std::cout << "unix_time: " << result.unix_time << std::endl;
        std::cout << "move_step_time: " << result.move_step_time << std::endl;
        std::cout << "data: " << std::endl;

        int time_index = 0;

        for (const auto& str : result.data) {
            // std::cout << "      " << str << std::endl;
            std::istringstream iss(str);
            std::string token;
            geometry_msgs::msg::PoseStamped pose;
            if (std::getline(iss, token, ':')) {
                pose.pose.position.x = std::stof(token) + 0.5; // 最初の部分をfloatに変換
            }

            if (std::getline(iss, token, ':')) {
                pose.pose.position.y = height - 1 - std::stof(token) + 0.5; // 次の部分をfloatに変換
            }
            pose.header.stamp.sec = time_index;
            time_index++;

            // std::cout << "(" << pose.pose.position.x << "," << pose.pose.position.y << ")" << std::endl;

            calc_msg->poses.push_back(pose);
        }
        calc_msg->poses.pop_back();

        rclcpp::sleep_for(std::chrono::seconds(5));

        path_publisher_->publish(*calc_msg);

        // timer_ = this->create_wall_timer(5000ms, [&]() { path_publisher_->publish(*calc_msg); });
    }
};
} // namespace tlab