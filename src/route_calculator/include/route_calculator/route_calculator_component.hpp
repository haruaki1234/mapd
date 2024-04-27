#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>
#include <nav_msgs/msg/grid_cells.hpp>

#include "a_star.hpp"
#include "g_layor.hpp"

#include <cmath>

namespace tlab
{

const size_t ROBOT_NUM = 2;

class RouteCalculator : public rclcpp::Node {
private:
    rclcpp::Subscription<my_msgs::msg::RouteRequest>::SharedPtr route_request_subscription;
    rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr infmap_subscription;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> calc_path_publisher_;

    std::vector<std::vector<int>> map_grid_matrix;
    // std::vector<std::vector<std::vector<std::vector<float>>>> g_foundation_list; // 156
    std::vector<std::vector<std::vector<float>>> g_foundation_list_defo;
    std::vector<std::vector<std::vector<float>>> g_foundation_list;
    std::vector<std::vector<float>> attentive_map;
    std::vector<std::vector<float>> obstacle_tile;
    std::vector<std::vector<float>> obstacle_MATnode;

    int64_t resolution;
    int64_t height_m;
    int64_t width_m;
    int64_t height_row;
    int64_t width_col;

    std::vector<float> pos;
    std::vector<int> MATpos;
    std::vector<float> node_MATpoint;
    float sqrt_2 = sqrt(2.0);
    ;
    float inflate_m;

    std::vector<float> start_;
    std::vector<float> goal_;

    Astar_Haru a_star_;
    G_Layor g_layor_;

    // posを行列に変換
    std::vector<int> to_MATpos_m(std::vector<float> pos, int height, int resolution)
    {
        float x = pos[0];
        float y = pos[1];
        int matC = round(x * resolution);
        int matR = round(height - y * resolution);
        std::vector<int> MATpos = {matR, matC};
        return MATpos;
    }

public:
    RouteCalculator(const rclcpp::NodeOptions& options) : RouteCalculator("", options) {}
    RouteCalculator(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_calcurator_node", name_space, options)
    {
        declare_parameter<int>("resolution", 1);
        resolution = get_parameter("resolution").as_int();
        declare_parameter<int>("height_m", 1);
        height_m = get_parameter("height_m").as_int();
        declare_parameter<int>("width_m", 1);
        width_m = get_parameter("width_m").as_int();
        declare_parameter<double>("inflate_m", 1.0);
        inflate_m = get_parameter("inflate_m").as_double();

        height_row = height_m * resolution;
        width_col = width_m * resolution;
        static int vec_row = height_row + 1;
        static int vec_col = width_col + 1;
        static float robot_size = inflate_m * 2;

        // g_foundation_list = g_layor_.GfoundList_createIni_4D(height_row, width_col, sqrt_2, ROBOT_NUM);//156
        g_foundation_list_defo = g_layor_.GfoundList_createIni(height_row, width_col, sqrt_2);

        float revision = (0.5 / resolution);

        // MAP情報をサブスクリプション
        infmap_subscription = this->create_subscription<nav_msgs::msg::GridCells>("infmap_topic_test", rclcpp::QoS(10), [&](const nav_msgs::msg::GridCells::SharedPtr infgrid_msg) {
            if (map_grid_matrix.size() != 0) {
                return;
            }

            map_grid_matrix = std::vector<std::vector<int>>(vec_row, std::vector<int>(vec_col, 0));
            for (size_t i = 0; i < infgrid_msg->cells.size(); i++) {
                pos = {infgrid_msg->cells[i].x, infgrid_msg->cells[i].y};
                MATpos = to_MATpos_m(pos, height_row, resolution);
                map_grid_matrix[MATpos[0]][MATpos[1]] = 1;
                // obstacle_tile.push_back(tile_point);
            }
        });

        // 各パスの送受信のための名前決め
        for (size_t i = 0; i < ROBOT_NUM; i++) {
            calc_path_publisher_.push_back(this->create_publisher<nav_msgs::msg::Path>("calc_path_" + std::to_string(i), rclcpp::QoS(10))); // topic name : "calc_path_0" , "calc_path_1" , ...
        }

        // route_request.msgを受信
        route_request_subscription = this->create_subscription<my_msgs::msg::RouteRequest>("route_request_topic_test", rclcpp::QoS(10), [&](const my_msgs::msg::RouteRequest::SharedPtr route_request_msg) {
            std::cout << "topic" << std::endl;
            auto route_calc_msg = std::make_shared<nav_msgs::msg::Path>();
            route_calc_msg->header.frame_id = "map"; // 基準座標を決めている、送るときだけ

            start_ = {route_request_msg->start.pose.position.x, route_request_msg->start.pose.position.y};
            goal_ = {route_request_msg->goal.pose.position.x, route_request_msg->goal.pose.position.y};

            // nav_msgs::msg::Path other_paths_;
            for (size_t i = 0; i < route_request_msg->other_path.size(); i++) {
                if (!route_request_msg->other_path[i].poses.empty()) {
                    for (size_t j = 0; j < route_request_msg->other_path[i].poses.size(); j++) {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.pose.position.x = route_request_msg->other_path[i].poses[j].pose.position.x;
                        pose.pose.position.y = route_request_msg->other_path[i].poses[j].pose.position.y;
                        std::cout << pose.pose.position.x << "  " << pose.pose.position.y << std::endl;
                        // other_paths_.poses.push_back(pose);
                    }
                }
            }

            std::cout << "topic3" << std::endl;
            g_foundation_list = g_layor_.renew_GfoundList(g_foundation_list_defo, height_row, width_col, resolution, robot_size, route_request_msg->other_path);
            // g_foundation_list = g_layor_.G_parton_Ini(g_foundation_list, route_request_msg->robot_no, height_row, width_col, sqrt_2);//156
            // g_foundation_list = g_layor_.GfoundList_other_robot_obstacle_4D(g_foundation_list, other_paths_, resolution, robot_size, route_request_msg->robot_no);//156
            // g_foundation_list = g_layor_.GfoundList_other_robot_obstacle(g_foundation_list, other_paths_, resolution, robot_size);
            std::cout << "topic4" << std::endl;
            std::cout << route_request_msg->robot_no << std::endl;
            auto path_result = a_star_.Astar_haru(start_, goal_, height_row, resolution, width_col, map_grid_matrix, g_foundation_list);
            std::cout << "topic5" << std::endl;
            std::vector<std::vector<float>> path = path_result.first;
            std::vector<std::vector<int>> MAT_path = path_result.second;

            geometry_msgs::msg::PoseStamped pose;
            for (size_t i = 0; i < path.size(); i++) {
                pose.pose.position.x = path[i][0];
                pose.pose.position.y = path[i][1];
                route_calc_msg->poses.push_back(pose);
            }
            // 該当するロボットのcalc_pathを送信
            std::cout << "topic6" << std::endl;
            calc_path_publisher_[route_request_msg->robot_no]->publish(*route_calc_msg);
        });
    }
};

} // namespace tlab

// rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
// rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription;
// rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;

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

// RCLCPP_INFO(this->get_logger(), "I heard: %lf", grid_msg->cell_width);

// // ループが終了した後にpathを表示
// std::cout << "Path: ";
// for (int i = 0; i < path.size(); ++i) {
//     std::cout << "(" << path[i][0] << ", " << path[i][1] << ") ";
// }
// std::cout << std::endl;

// RCLCPP_INFO(this->get_logger(), "I heard: %lf", route_request_msg->goal.pose.position.x);

// route_calc_msg->poses.push_back(route_request_msg->start);
// route_calc_msg->poses.push_back(route_request_msg->goal);

// for (size_t i = 0; i < obstacle_tile.size(); i++) {
//     node_MATpoint = {(height_row - ((obstacle_tile[i][1] - revision) * resolution)), (obstacle_tile[i][0] - revision) * resolution}; //{row,col}
//     obstacle_MATnode.push_back(node_MATpoint);
//     node_MATpoint = {(height_row - ((obstacle_tile[i][1] + revision) * resolution)), (obstacle_tile[i][0] - revision) * resolution}; //{row,col}
//     obstacle_MATnode.push_back(node_MATpoint);
// }
// node_MATpoint = {(height_row - ((obstacle_tile.back()[1] - revision) * resolution)), (obstacle_tile.back()[0] + revision) * resolution}; //{row,col}
// obstacle_MATnode.push_back(node_MATpoint);
// node_MATpoint = {(height_row - ((obstacle_tile.back()[1] + revision) * resolution)), (obstacle_tile.back()[0] + revision) * resolution}; //{row,col}
// obstacle_MATnode.push_back(node_MATpoint);

// for (size_t i = 0; i < obstacle_MATnode.size(); i++) {
//     if (0 <= obstacle_MATnode[i][0] && obstacle_MATnode[i][0] < vec_row && 0 <= obstacle_MATnode[i][1] && obstacle_MATnode[i][1] < vec_col) {
//         map_grid_matrix[obstacle_MATnode[i][0]][obstacle_MATnode[i][1]] = 1;
//     }
//     else {
//     }
// }