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
    int64_t resolution;
    int64_t height_m;
    int64_t width_m;
    float inflate_m;

    std::vector<float> map_terminal;
    std::vector<std::vector<float>> map_terminals;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr grid_publisher_;
    nav_msgs::msg::GridCells::SharedPtr grid_msg_;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr infgrid_publisher_;
    nav_msgs::msg::GridCells::SharedPtr infgrid_msg_;

    std::vector<float> create_visual_map(float x1, float x2, float y1, float y2, float resolution)
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
        std::vector<float> map_terminal = {x1, x2, y1, y2};
        return map_terminal;
    }

    void create_visual_infmap(float x1, float x2, float y1, float y2, float resolution, float inflate_m, float height_m, float width_m)
    {
        x1 = x1 - inflate_m;
        x2 = x2 + inflate_m;
        y1 = y1 - inflate_m;
        y2 = y2 + inflate_m;

        if (x1 < 0) {
            x1 = 0;
        }
        if (y1 < 0) {
            y1 = 0;
        }
        if (x2 > width_m) {
            x2 = width_m;
        }
        if (y2 > height_m) {
            y2 = height_m;
        }

        float diff_x = (x2 - x1) * resolution;
        float diff_y = (y2 - y1) * resolution;
        for (size_t i = 0; i <= diff_x; i++) {
            for (size_t j = 0; j <= diff_y; j++) {
                geometry_msgs::msg::Point infgrider;
                infgrider.x = (i / resolution) + x1;
                infgrider.y = (j / resolution) + y1;
                infgrider.z = 0;
                infgrid_msg_->cells.push_back(infgrider);
            }
        }
    }

public:
    MapPublisher(const rclcpp::NodeOptions& options) : MapPublisher("", options) {}
    MapPublisher(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("map_publisher_node", name_space, options), grid_msg_(std::make_shared<nav_msgs::msg::GridCells>()), infgrid_msg_(std::make_shared<nav_msgs::msg::GridCells>())
    {
        using namespace std::chrono_literals;

        declare_parameter<int>("resolution", 1);
        resolution = get_parameter("resolution").as_int();
        declare_parameter<int>("height_m", 1);
        height_m = get_parameter("height_m").as_int();
        declare_parameter<int>("width_m", 1);
        width_m = get_parameter("width_m").as_int();
        declare_parameter<double>("inflate_m", 0.3);
        inflate_m = get_parameter("inflate_m").as_double();

        grid_msg_->header.frame_id = "map";
        grid_msg_->cell_width = 1.0 / resolution;
        grid_msg_->cell_height = 1.0 / resolution;

        infgrid_msg_->header.frame_id = "map";
        infgrid_msg_->cell_width = 1.0 / resolution;
        infgrid_msg_->cell_height = 1.0 / resolution;

        grid_publisher_ = this->create_publisher<nav_msgs::msg::GridCells>("map_topic_test", rclcpp::QoS(10));
        infgrid_publisher_ = this->create_publisher<nav_msgs::msg::GridCells>("infmap_topic_test", rclcpp::QoS(10));

        map_terminals.push_back(create_visual_map(0, 37, 0, 1, resolution));
        map_terminals.push_back(create_visual_map(0, 37, 18, 19, resolution));
        map_terminals.push_back(create_visual_map(0, 1, 1, 18, resolution));
        map_terminals.push_back(create_visual_map(36, 37, 1, 18, resolution));

        create_visual_infmap(0, 37, 0, 1, resolution, inflate_m, height_m, width_m);
        create_visual_infmap(0, 37, 18, 19, resolution, inflate_m, height_m, width_m);
        create_visual_infmap(0, 1, 1, 18, resolution, inflate_m, height_m, width_m);
        create_visual_infmap(36, 37, 1, 18, resolution, inflate_m, height_m, width_m);

        map_terminals.push_back(create_visual_map(6, 16, 6, 8, resolution));
        map_terminals.push_back(create_visual_map(6, 16, 11, 13, resolution));
        map_terminals.push_back(create_visual_map(21, 31, 6, 8, resolution));
        map_terminals.push_back(create_visual_map(21, 31, 11, 13, resolution));

        create_visual_infmap(6, 16, 6, 8, resolution, inflate_m, height_m, width_m);
        create_visual_infmap(6, 16, 11, 13, resolution, inflate_m, height_m, width_m);
        create_visual_infmap(21, 31, 6, 8, resolution, inflate_m, height_m, width_m);
        create_visual_infmap(21, 31, 11, 13, resolution, inflate_m, height_m, width_m);

        timer_ = this->create_wall_timer(1000ms, [&]() {
            grid_publisher_->publish(*grid_msg_);
            infgrid_publisher_->publish(*infgrid_msg_);
        });
    }
};

} // namespace tlab