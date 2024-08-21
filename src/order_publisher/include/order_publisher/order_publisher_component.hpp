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
#include <my_msgs/msg/pick_request.hpp>
#include <my_msgs/msg/picks.hpp>
#include <my_msgs/msg/re_order.hpp>
#include <chrono>
#include <random>
#include <algorithm>

namespace tlab
{

const size_t ROBOT_NUM = 20;
const int one_order_pick_num = 4;

struct Table {
    std::vector<float> xy_left;
    std::vector<float> xy_right;
    double theta;
    std::string label;
    int pari;
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
    int order_number = 100;
    int rand_seed = 6432;

    std::vector<std::vector<float>> start_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> goal_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));

    rclcpp::Publisher<my_msgs::msg::OrderRequest>::SharedPtr order_request_publisher_;
    rclcpp::Publisher<my_msgs::msg::PickRequest>::SharedPtr pick_point_publisher_;
    rclcpp::Publisher<my_msgs::msg::Picks>::SharedPtr picks_publisher_;
    rclcpp::Subscription<my_msgs::msg::ReOrder>::SharedPtr re_order_subscription;

    std::string shelfAlpha = "ABCDEFGHI";
    int capacity = 100;
    std::map<char, int> table_up;
    std::map<char, std::vector<Table>> table_list;

    std::vector<std::string> picks;
    std::vector<std::vector<std::string>> orders_100;

    void initializeTables(std::map<char, int>& table_up, std::map<char, std::vector<Table>>& table_list, const std::string& shelfAlpha, int capacity)
    {
        for (char letter : shelfAlpha) {
            table_up[letter] = 0;
            table_list[letter] = std::vector<Table>(capacity, {{0, 0}, {0, 0}, 0, std::string(1, letter), 0});
        }
    }

    // 横に複数生成することを想定、下(1),上(2)
    void shelf_create(double X1, double X2, double Y1, double Y2, char sign, int leri, double shelf_theta, double s_width, double s_height, float height_m, float width_m, int resolution, double inflate_m, std::map<char, int>& table_up, std::map<char, std::vector<Table>>& table_list, std::shared_ptr<my_msgs::msg::PickRequest_<std::allocator<void>>>& pick_point_msg, std::shared_ptr<my_msgs::msg::Picks_<std::allocator<void>>>& picks_msg, std::vector<std::string>& picks) //, std::shared_ptr<nav_msgs::msg::Path_<std::allocator<void>>>& pick_point_msg
    {
        int create_num = round((X2 - X1) / s_width);
        double shelf_X_left, shelf_X_right, shelf_Y;
        std::string combined;

        if (leri == 1) {
            shelf_X_left = X1 + s_width / 4;
            shelf_X_right = X1 + 3 * s_width / 4;
            shelf_Y = Y1 - inflate_m - 0.1;
            for (int i = 0; i < create_num; i++) {
                Table table;
                table.xy_left = {shelf_X_left + s_width * i, shelf_Y};
                table.xy_right = {shelf_X_right + s_width * i, shelf_Y};
                table.pari = leri;

                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = shelf_X_left + s_width * i;
                pose.pose.position.y = shelf_Y;
                pick_point_msg->down_pick.poses.push_back(pose);

                pose.pose.position.x = shelf_X_right + s_width * i;
                pick_point_msg->down_pick.poses.push_back(pose);

                table.theta = shelf_theta;
                table.label = sign;
                table_list[sign][table_up[sign]] = table;
                combined = std::string(1, sign) + std::to_string(table_up[sign]);
                picks.push_back(combined);
                picks_msg->all_picks.push_back(combined);
                table_up[sign]++;
            }
        }
        else if (leri == 2) {
            shelf_X_left = X1 + s_width / 4;
            shelf_X_right = X1 + 3 * s_width / 4;
            shelf_Y = Y2 + inflate_m + 0.1;
            for (int i = 0; i < create_num; i++) {
                Table table;
                table.xy_left = {shelf_X_left + s_width * i, shelf_Y};
                table.xy_right = {shelf_X_right + s_width * i, shelf_Y};
                table.pari = leri;

                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = shelf_X_left + s_width * i;
                pose.pose.position.y = shelf_Y;
                pick_point_msg->up_pick.poses.push_back(pose);

                pose.pose.position.x = shelf_X_right + s_width * i;
                pick_point_msg->up_pick.poses.push_back(pose);

                table.theta = shelf_theta;
                table.label = sign;
                table_list[sign][table_up[sign]] = table;
                combined = std::string(1, sign) + std::to_string(table_up[sign]);
                picks.push_back(combined);
                picks_msg->all_picks.push_back(combined);
                table_up[sign]++;
            }
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
        start_.pose.position.z = 0; // パリティ用
        order_request_msg->order_paths.poses.push_back(start_);

        for (int i = 0; i < send_order_path_.size(); i++) {
            std::string atten_all_letter = send_order_path_[i];
            char letter = atten_all_letter[0];
            int number;
            std::string number_str = atten_all_letter.substr(1); // 数字部分のサブストリングを取得
            std::stringstream(number_str) >> number;
            // path型に代入していこう
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = table_list[letter][number].xy_left[0];
            pose.pose.position.y = table_list[letter][number].xy_left[1];
            pose.pose.position.z = table_list[letter][number].pari; // パリティ用
            order_request_msg->order_paths.poses.push_back(pose);
            pose.pose.position.x = table_list[letter][number].xy_right[0];
            pose.pose.position.y = table_list[letter][number].xy_right[1];
            order_request_msg->order_paths.poses.push_back(pose);
            order_request_msg->pick_4.push_back(send_order_path_[i]);
        }
        // goalの位置
        geometry_msgs::msg::PoseStamped goal_;
        goal_.pose.position.x = goal[0];
        goal_.pose.position.y = goal[1];
        goal_.pose.position.z = 0; // パリティ用
        order_request_msg->order_paths.poses.push_back(goal_);
        order_request_msg->order_paths.poses.push_back(start_);

        order_request_publisher_->publish(*order_request_msg);
    }

    std::vector<std::string> generate4Pick(std::vector<std::string>& picks, int seed)
    {
        // ランダムにシャッフル
        // std::random_device rd;
        // std::mt19937 g(rd());
        std::mt19937 g(seed);
        shuffle(picks.begin(), picks.end(), g);

        // 最初の4つを取り出して返す
        std::vector<std::string> result(picks.begin(), picks.begin() + one_order_pick_num);
        return result;
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

        start_pose[0] = {2.0, 2.0};
        goal_pose[0] = {38.0, 2.0};
        start_pose[1] = {4.0, 2.0};
        goal_pose[1] = {40.0, 2.0};
        start_pose[2] = {6.0, 2.0};
        goal_pose[2] = {42.0, 2.0};
        start_pose[3] = {8.0, 2.0};
        goal_pose[3] = {44.0, 2.0};
        start_pose[4] = {10.0, 2.0};
        goal_pose[4] = {46.0, 2.0};
        start_pose[5] = {2.0, 4.0};
        goal_pose[5] = {38.0, 4.0};
        start_pose[6] = {4.0, 4.0};
        goal_pose[6] = {40.0, 4.0};
        start_pose[7] = {6.0, 4.0};
        goal_pose[7] = {42.0, 4.0};
        start_pose[8] = {8.0, 4.0};
        goal_pose[8] = {44.0, 4.0};
        start_pose[9] = {10.0, 4.0};
        goal_pose[9] = {46.0, 4.0};
        start_pose[10] = {2.0, 6.0};
        goal_pose[10] = {38.0, 6.0};
        start_pose[11] = {4.0, 6.0};
        goal_pose[11] = {40.0, 6.0};
        start_pose[12] = {6.0, 6.0};
        goal_pose[12] = {42.0, 6.0};
        start_pose[13] = {8.0, 6.0};
        goal_pose[13] = {44.0, 6.0};
        start_pose[14] = {10.0, 6.0};
        goal_pose[14] = {46.0, 6.0};
        start_pose[15] = {2.0, 8.0};
        goal_pose[15] = {38.0, 8.0};
        start_pose[16] = {4.0, 8.0};
        goal_pose[16] = {40.0, 8.0};
        start_pose[17] = {6.0, 8.0};
        goal_pose[17] = {42.0, 8.0};
        start_pose[18] = {8.0, 8.0};
        goal_pose[18] = {44.0, 8.0};
        start_pose[19] = {10.0, 8.0};
        goal_pose[19] = {46.0, 8.0};

        initializeTables(table_up, table_list, shelfAlpha, capacity);

        // pick_point_msgをpublish
        auto pick_point_msg = std::make_shared<my_msgs::msg::PickRequest>();
        pick_point_msg->path_num = order_number * (one_order_pick_num + 2);
        auto picks_msg = std::make_shared<my_msgs::msg::Picks>();
        pick_point_publisher_ = this->create_publisher<my_msgs::msg::PickRequest>("pick_point_topic_test", rclcpp::QoS(10));
        picks_publisher_ = this->create_publisher<my_msgs::msg::Picks>("picks", rclcpp::QoS(10));
        for (int i = 0; i < ROBOT_NUM; i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start_pose[i][0];
            pose.pose.position.y = start_pose[i][1];
            pick_point_msg->start_goal.poses.push_back(pose);
            pose.pose.position.x = goal_pose[i][0];
            pose.pose.position.y = goal_pose[i][1];
            pick_point_msg->start_goal.poses.push_back(pose);
        }

        shelf_create(6, 16, 12, 13, 'A', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(6, 16, 13, 14, 'A', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(6, 16, 17, 18, 'B', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(6, 16, 18, 19, 'B', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(6, 16, 22, 23, 'C', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(6, 16, 23, 24, 'C', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);

        shelf_create(20, 30, 12, 13, 'D', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(20, 30, 13, 14, 'D', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(20, 30, 17, 18, 'E', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(20, 30, 18, 19, 'E', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(20, 30, 22, 23, 'F', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(20, 30, 23, 24, 'F', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);

        shelf_create(34, 44, 12, 13, 'G', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(34, 44, 13, 14, 'G', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(34, 44, 17, 18, 'H', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(34, 44, 18, 19, 'H', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(34, 44, 22, 23, 'I', 1, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);
        shelf_create(34, 44, 23, 24, 'I', 2, shelf_theta, s_width, s_height, height_m, width_m, resolution, inflate_m, table_up, table_list, pick_point_msg, picks_msg, picks);

        for (int i = 0; i < order_number; i++) {
            std::vector<std::string> pick_4 = generate4Pick(picks, rand_seed + i);
            orders_100.push_back(pick_4);
            // for (int j = 0; j < pick_4.size(); j++) {
            //     std::cout << pick_4[j] << " ";
            // }
            // std::cout << std::endl;
        }

        pick_point_publisher_->publish(*pick_point_msg);
        picks_publisher_->publish(*picks_msg);
        rclcpp::sleep_for(std::chrono::seconds(5));
        for (int i = 0; i < ROBOT_NUM; i++) {
            {
                request_order(i, orders_100[i], start_pose[i], goal_pose[i]);
            }

            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        // 再オーダ用
        re_order_subscription = this->create_subscription<my_msgs::msg::ReOrder>("reorder", rclcpp::QoS(10), [&](const my_msgs::msg::ReOrder::SharedPtr re_order_msg) { request_order(re_order_msg->robot_no, orders_100[re_order_msg->order_no], start_pose[re_order_msg->robot_no], goal_pose[re_order_msg->robot_no]); });

        // rclcpp::sleep_for(std::chrono::seconds(10));
        // {
        //     request_order(2, {"E1", "G3", "D2", "A1"}, start_pose[2], goal_pose[2]); // {"E1", "A3", "D2", "H1"}
        // }

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