#pragma once

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/order_request.hpp>

namespace tlab
{

struct Table {
    std::vector<float> xy;
    double theta;
    std::string label;
};

class OrderPublisher : public rclcpp::Node {
private:
    int64_t resolution;
    int64_t height_m;
    int64_t width_m;
    double inflate_m;
    double s_width = 2;
    double s_height = 1;
    double shelf_theta = M_PI / 2;
    int leri;

    rclcpp::Publisher<my_msgs::msg::OrderRequest>::SharedPtr order_request_publisher_;

    std::string shelfAlpha = "ABCDEFGH";
    int capacity = 100;
    std::map<char, int> table_up;
    std::map<char, std::vector<Table>> table_list;

    void initializeTables(std::map<char, int>& table_up, std::map<char, std::vector<Table>>& table_list, const std::string& shelfAlpha, int capacity)
    {
        for (char letter : shelfAlpha) {
            table_up[letter] = 0;
            table_list[letter] = std::vector<Table>(capacity, {{0, 0}, 0, std::string(1, letter)});
        }
    }

    // 横に複数生成することを想定、下(1),上(2)
    void shelf_create(double X1, double X2, double Y1, double Y2, char sign, int leri, double shelf_theta, double s_width, double s_height, float height_m, float width_m, int resolution, double inflate_m, std::map<char, int>& table_up, std::map<char, std::vector<Table>>& table_list)
    {
        int create_num = round((X2 - X1) / s_width);
        double shelf_X, shelf_Y;

        if (leri == 1) {
            shelf_X = X1 + s_width / 2;
            shelf_Y = Y1 - inflate_m - 0.1;
        }
        else if (leri == 2) {
            shelf_X = X1 + s_width / 2;
            shelf_Y = Y2 + inflate_m + 0.1;
        }

        for (int i = 0; i < create_num; i++) {
            Table table;
            table.xy = {shelf_X + s_width * i, shelf_Y};
            table.theta = shelf_theta;
            table.label = sign;
            table_list[sign][table_up[sign]] = table;
            table_up[sign]++;
        }
    }

    // Order注文
    void request_order(int send_no, std::vector<std::string> send_order_path_, std::vector<float> start, std::vector<float> goal)
    {
        auto order_request_msg = std::make_shared<my_msgs::msg::OrderRequest>();
        order_request_publisher_ = this->create_publisher<my_msgs::msg::OrderRequest>("order_request_topic_test", rclcpp::QoS(10));
        order_request_msg->robot_no = send_no;
        // startの位置
        geometry_msgs::msg::PoseStamped start_;
        start_.pose.position.x = start[0];
        start_.pose.position.y = start[1];
        order_request_msg->order_paths.poses.push_back(start_);

        for (int i = 0; i < send_order_path_.size(); i++) {
            std::string atten_all_letter = send_order_path_[i];
            char letter = atten_all_letter[0];
            int number;
            std::string number_str = atten_all_letter.substr(1); // 数字部分のサブストリングを取得
            std::stringstream(number_str) >> number;
            // path型に代入していこう
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = table_list[letter][number].xy[0];
            pose.pose.position.y = table_list[letter][number].xy[1];
            // std::cout << pose.pose.position.x << std::endl;
            // std::cout << pose.pose.position.y << std::endl;

            order_request_msg->order_paths.poses.push_back(pose);
        }
        // goalの位置
        geometry_msgs::msg::PoseStamped goal_;
        goal_.pose.position.x = goal[0];
        goal_.pose.position.y = goal[1];
        order_request_msg->order_paths.poses.push_back(goal_);
        order_request_publisher_->publish(*order_request_msg);
    }

public:
    OrderPublisher(const rclcpp::NodeOptions& options) : OrderPublisher("", options) {}
    OrderPublisher(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("order_publisher_node", name_space, options)
    {
        declare_parameter<int>("resolution", 1);
        resolution = get_parameter("resolution").as_int();
        declare_parameter<int>("height_m", 1);
        height_m = get_parameter("height_m").as_int();
        declare_parameter<int>("width_m", 1);
        width_m = get_parameter("width_m").as_int();
        declare_parameter<double>("inflate_m", 0.3);
        inflate_m = get_parameter("inflate_m").as_double();

        initializeTables(table_up, table_list, shelfAlpha, capacity);

        shelf_create(6, 16, 6, 7, 'A', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);
        shelf_create(6, 16, 7, 8, 'B', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);
        shelf_create(6, 16, 11, 12, 'C', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);
        shelf_create(6, 16, 12, 13, 'D', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);
        shelf_create(21, 31, 6, 7, 'E', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);
        shelf_create(21, 31, 7, 8, 'F', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);
        shelf_create(21, 31, 11, 12, 'G', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);
        shelf_create(21, 31, 12, 13, 'H', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list);

        {
            request_order(0, {"A1", "B2"}, {2.0, 2.0}, {34.0, 2.0});
        }
        rclcpp::sleep_for(std::chrono::seconds(3));
        {
            request_order(1, {"H2", "A0"}, {6.0, 2.0}, {30.0, 2.0});
        }

        // {
        //     int send_no = 1;
        //     std::vector<std::string> send_order_path_{"H2", "A0"}; //, "C1", "D2", "H3"
        //     auto order_request_msg = std::make_shared<my_msgs::msg::OrderRequest>();
        //     order_request_publisher_ = this->create_publisher<my_msgs::msg::OrderRequest>("order_request_topic_test", rclcpp::QoS(10));
        //     order_request_msg->robot_no = send_no;
        //     // startの位置
        //     geometry_msgs::msg::PoseStamped start_;
        //     start_.pose.position.x = 6;
        //     start_.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(start_);

        //     for (int i = 0; i < send_order_path_.size(); i++) {
        //         std::string atten_all_letter = send_order_path_[i];
        //         char letter = atten_all_letter[0];
        //         int number;
        //         std::string number_str = atten_all_letter.substr(1); // 数字部分のサブストリングを取得
        //         std::stringstream(number_str) >> number;
        //         // path型に代入していこう
        //         geometry_msgs::msg::PoseStamped pose;
        //         pose.pose.position.x = table_list[letter][number].xy[0];
        //         pose.pose.position.y = table_list[letter][number].xy[1];
        //         // std::cout << pose.pose.position.x << std::endl;
        //         // std::cout << pose.pose.position.y << std::endl;

        //         order_request_msg->order_paths.poses.push_back(pose);
        //     }
        //     // goalの位置
        //     geometry_msgs::msg::PoseStamped goal;
        //     goal.pose.position.x = 30;
        //     goal.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(goal);
        //     order_request_publisher_->publish(*order_request_msg);
        // }

        // rclcpp::sleep_for(std::chrono::seconds(3));

        // {
        //     int send_no = 2;
        //     std::vector<std::string> send_order_path_{"D1", "E1"}; //, "G2", "G4", "B0"
        //     auto order_request_msg = std::make_shared<my_msgs::msg::OrderRequest>();
        //     order_request_publisher_ = this->create_publisher<my_msgs::msg::OrderRequest>("order_request_topic_test", rclcpp::QoS(10));
        //     order_request_msg->robot_no = send_no;
        //     // startの位置
        //     geometry_msgs::msg::PoseStamped start_;
        //     start_.pose.position.x = 10;
        //     start_.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(start_);

        //     for (int i = 0; i < send_order_path_.size(); i++) {
        //         std::string atten_all_letter = send_order_path_[i];
        //         char letter = atten_all_letter[0];
        //         int number;
        //         std::string number_str = atten_all_letter.substr(1); // 数字部分のサブストリングを取得
        //         std::stringstream(number_str) >> number;
        //         // path型に代入していこう
        //         geometry_msgs::msg::PoseStamped pose;
        //         pose.pose.position.x = table_list[letter][number].xy[0];
        //         pose.pose.position.y = table_list[letter][number].xy[1];
        //         // std::cout << pose.pose.position.x << std::endl;
        //         // std::cout << pose.pose.position.y << std::endl;

        //         order_request_msg->order_paths.poses.push_back(pose);
        //     }
        //     // goalの位置
        //     geometry_msgs::msg::PoseStamped goal;
        //     goal.pose.position.x = 26;
        //     goal.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(goal);
        //     order_request_publisher_->publish(*order_request_msg);
        // }

        // rclcpp::sleep_for(std::chrono::seconds(3));

        // {
        //     int send_no = 3;
        //     std::vector<std::string> send_order_path_{"D4", "G0", "F0"};
        //     auto order_request_msg = std::make_shared<my_msgs::msg::OrderRequest>();
        //     order_request_publisher_ = this->create_publisher<my_msgs::msg::OrderRequest>("order_request_topic_test", rclcpp::QoS(10));
        //     order_request_msg->robot_no = send_no;
        //     // startの位置
        //     geometry_msgs::msg::PoseStamped start_;
        //     start_.pose.position.x = 14;
        //     start_.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(start_);

        //     for (int i = 0; i < send_order_path_.size(); i++) {
        //         std::string atten_all_letter = send_order_path_[i];
        //         char letter = atten_all_letter[0];
        //         int number;
        //         std::string number_str = atten_all_letter.substr(1); // 数字部分のサブストリングを取得
        //         std::stringstream(number_str) >> number;
        //         // path型に代入していこう
        //         geometry_msgs::msg::PoseStamped pose;
        //         pose.pose.position.x = table_list[letter][number].xy[0];
        //         pose.pose.position.y = table_list[letter][number].xy[1];
        //         // std::cout << pose.pose.position.x << std::endl;
        //         // std::cout << pose.pose.position.y << std::endl;

        //         order_request_msg->order_paths.poses.push_back(pose);
        //     }
        //     // goalの位置
        //     geometry_msgs::msg::PoseStamped goal;
        //     goal.pose.position.x = 22;
        //     goal.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(goal);
        //     order_request_publisher_->publish(*order_request_msg);
        // }

        // {
        //     int send_no = 4;
        //     std::vector<std::string> send_order_path_{"B1", "D0", "F4"};
        //     auto order_request_msg = std::make_shared<my_msgs::msg::OrderRequest>();
        //     order_request_publisher_ = this->create_publisher<my_msgs::msg::OrderRequest>("order_request_topic_test", rclcpp::QoS(10));
        //     order_request_msg->robot_no = send_no;
        //     // startの位置
        //     geometry_msgs::msg::PoseStamped start_;
        //     start_.pose.position.x = 14;
        //     start_.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(start_);

        //     for (int i = 0; i < send_order_path_.size(); i++) {
        //         std::string atten_all_letter = send_order_path_[i];
        //         char letter = atten_all_letter[0];
        //         int number;
        //         std::string number_str = atten_all_letter.substr(1); // 数字部分のサブストリングを取得
        //         std::stringstream(number_str) >> number;
        //         // path型に代入していこう
        //         geometry_msgs::msg::PoseStamped pose;
        //         pose.pose.position.x = table_list[letter][number].xy[0];
        //         pose.pose.position.y = table_list[letter][number].xy[1];
        //         // std::cout << pose.pose.position.x << std::endl;
        //         // std::cout << pose.pose.position.y << std::endl;

        //         order_request_msg->order_paths.poses.push_back(pose);
        //     }
        //     // goalの位置
        //     geometry_msgs::msg::PoseStamped goal;
        //     goal.pose.position.x = 22;
        //     goal.pose.position.y = 2;
        //     order_request_msg->order_paths.poses.push_back(goal);
        //     order_request_publisher_->publish(*order_request_msg);
        // }
    }
};

} // namespace tlab