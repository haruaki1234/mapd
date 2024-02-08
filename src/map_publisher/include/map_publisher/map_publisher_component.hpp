#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace tlab
{

class MapPublisher : public rclcpp::Node {
private:
    int64_t resolution = 10;
    // int64_t height = 5;
    // int64_t width = 10;

    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr grid_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::GridCells::SharedPtr grid_msg_;

    int add(int a, int b) { return a + b; }

    void create_visual_map(float x1, float x2, float y1, float y2, float resolution)
    {
        float diff_x = (x2 - x1) * resolution;
        float diff_y = (y2 - y1) * resolution;
        float revision = (0.5 / resolution);
        for (size_t i = 0; i < diff_x; i++) {
            for (size_t j = 0; j < diff_y; j++) {
                geometry_msgs::msg::Point grider;
                grider.x = (i / resolution) + x1 + revision;
                grider.y = (j / resolution) + y1 + revision;
                grider.z = 0;
                grid_msg_->cells.push_back(grider);
            }
        }
    }

public:
    MapPublisher(const rclcpp::NodeOptions& options) : MapPublisher("", options) {}
    MapPublisher(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("map_publisher_node", name_space, options), grid_msg_(std::make_shared<nav_msgs::msg::GridCells>())
    {
        using namespace std::chrono_literals;

        grid_msg_->header.frame_id = "map";
        grid_msg_->cell_width = 1.0 / resolution;
        grid_msg_->cell_height = 1.0 / resolution;

        create_visual_map(0, 37, 0, 1, resolution);
        create_visual_map(0, 37, 18, 19, resolution);
        create_visual_map(0, 1, 1, 18, resolution);
        create_visual_map(36, 37, 1, 18, resolution);

        create_visual_map(6, 16, 6, 8, resolution);
        create_visual_map(6, 16, 11, 13, resolution);
        create_visual_map(21, 31, 6, 8, resolution);
        create_visual_map(21, 31, 11, 13, resolution);

        grid_publisher_ = this->create_publisher<nav_msgs::msg::GridCells>("grid_topic_test", rclcpp::QoS(10));

        timer_ = this->create_wall_timer(500ms, [&]() { grid_publisher_->publish(*grid_msg_); });
    }
};

} // namespace tlab