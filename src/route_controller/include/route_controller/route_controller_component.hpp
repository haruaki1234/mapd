#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>
#include <my_msgs/msg/order_request.hpp>
#include <my_msgs/msg/re_order.hpp>
#include <my_msgs/msg/picks.hpp>
#include <my_msgs/msg/re_route_request.hpp>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>

namespace tlab
{

struct picks_system {
    std::vector<std::string> all_pickings;
    std::vector<int> left_condition;
    std::vector<int> right_condition;
};
const size_t ROBOT_NUM = 20;
const int taiki_time = 10;

class RouteController : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> vec_pose_publisher_;
    rclcpp::Publisher<my_msgs::msg::RouteRequest>::SharedPtr route_request_publisher_;
    rclcpp::Publisher<my_msgs::msg::ReOrder>::SharedPtr re_order_publisher_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> calc_path_subscription_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> calc_MATpath_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr attentive_path_subscription;
    rclcpp::Subscription<my_msgs::msg::OrderRequest>::SharedPtr order_request_subscription;
    rclcpp::Subscription<my_msgs::msg::ReRouteRequest>::SharedPtr re_route_request_subscription;
    rclcpp::Subscription<my_msgs::msg::Picks>::SharedPtr picks_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr point_timer_;

    size_t count_;
    int csv_one_count = 0;
    int conf_count = 0;
    std::vector<int> path_array_num_ = std::vector<int>(ROBOT_NUM, 0);
    std::vector<int> order_array_num_ = std::vector<int>(ROBOT_NUM, 0);
    std::vector<int> str_pick_array_num_ = std::vector<int>(ROBOT_NUM, 0);
    std::vector<bool> picking_condition = std::vector<bool>(ROBOT_NUM, true);
    std::vector<std::vector<float>> path_attentive_last_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));

    nav_msgs::msg::Path defaultPath;

    std::vector<nav_msgs::msg::Path> all_robot_path;
    std::vector<nav_msgs::msg::Path> all_robot_MATpath;
    std::vector<nav_msgs::msg::Path> sub_order_paths = std::vector<nav_msgs::msg::Path>(ROBOT_NUM);
    std::vector<int> pre_pick_index_retention = std::vector<int>(ROBOT_NUM);
    std::vector<int> pre_pre_pick_index_retention = std::vector<int>(ROBOT_NUM, -1);
    std::vector<int> pre_pre_pick_left_right_retention = std::vector<int>(ROBOT_NUM, -1);
    std::vector<int> pre_pick_left_right_retention = std::vector<int>(ROBOT_NUM); // 1はleft、2は右
    std::vector<bool> odds_key = std::vector<bool>(ROBOT_NUM, true);
    std::vector<std::vector<std::string>> sub_order_pick4 = std::vector<std::vector<std::string>>(ROBOT_NUM, std::vector<std::string>(4));
    int order_100_count = 0;

    std::vector<int> robot_mode = std::vector<int>(ROBOT_NUM, 4);
    std::vector<std::vector<float>> current_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> next_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> diff_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> current_move_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> next_move_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));

    std::vector<std::pair<float, float>> past_pairs = std::vector<std::pair<float, float>>(ROBOT_NUM, std::pair<float, float>(-1, -1));
    std::vector<std::pair<float, float>> current_pairs = std::vector<std::pair<float, float>>(ROBOT_NUM, std::pair<float, float>(0, 0));
    int conflict_count = -ROBOT_NUM;
    // int taiki_time = 10;

    picks_system picks_sys;

    float inv_route_2 = 1 / std::sqrt(2.0);
    float v = 1; //[m/s]
    std::vector<float> stop_count = std::vector<float>(ROBOT_NUM, 0);
    std::vector<float> mode1_count = std::vector<float>(ROBOT_NUM, 0);
    std::vector<float> mode3_count = std::vector<float>(ROBOT_NUM, 0);
    std::vector<float> mode4_count = std::vector<float>(ROBOT_NUM, 0);
    std::vector<float> mode5_count = std::vector<float>(ROBOT_NUM, 0);
    std::vector<std::vector<float>> mode_condition = std::vector<std::vector<float>>(ROBOT_NUM);
    std::vector<std::vector<float>> mode_condition_time = std::vector<std::vector<float>>(ROBOT_NUM);
    std::string condition_csv_path = "/home/haru/ros2_ws/mode_condition";
    std::string condition_time_csv_path = "/home/haru/ros2_ws/mode_condition_time";

    std::vector<int> path_plan_next = std::vector<int>(ROBOT_NUM, -1);
    int path_plan_now = -1;
    int path_plan_first = -1;
    // int void_count;

    void writeCSV_A(const std::string& path, const std::vector<std::vector<float>>& output, const std::string& name)
    {
        // 現在の日時を取得し、ファイル名に使用する
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm;
#ifdef _WIN32
        localtime_s(&now_tm, &now_time);
#else
        localtime_r(&now_time, &now_tm);
#endif
        std::ostringstream oss;
        oss << name << std::put_time(&now_tm, "%Y年%m月%d日%H時%M分%S秒") << ".csv";
        std::string filename = oss.str();

        // ファイルのフルパスを生成
        std::string filepath = path + "/" + filename;

        std::ofstream file(filepath);

        if (!file.is_open()) {
            std::cerr << "Error: Unable to open file " << filepath << std::endl;
            return;
        }

        // データをCSV形式でファイルに書き込む
        for (const auto& row : output) {
            for (size_t i = 0; i < row.size(); ++i) {
                file << row[i];
                // 最後の要素でなければ、コンマを挿入
                if (i != row.size() - 1)
                    file << ",";
            }
            // 行を終端するために改行を挿入
            file << std::endl;
        }

        file.close();

        std::cout << "CSVファイルが作成されました: " << filepath << std::endl;
    }

    // 距離を計算する関数
    // float calculateDistance(const std::vector<float>& coord1, const std::vector<float>& coord2) { return std::sqrt(std::pow(coord1[0] - coord2[0], 2) + std::pow(coord1[1] - coord2[1], 2)); }
    float calculateDistance(const std::vector<float>& coord1, const std::vector<float>& coord2) { return std::hypot(coord1[0] - coord2[0], coord1[1] - coord2[1]); }

    void create_time_skip_vec(std::shared_ptr<my_msgs::msg::RouteRequest>& route_request_msg, const std::vector<nav_msgs::msg::Path>& all_robot_MATpath, const std::vector<int>& path_array_num_, const std::vector<int>& robot_mode, const std::vector<float>& stop_count)
    {
        for (int k = 0; k < ROBOT_NUM; k++) {
            if (route_request_msg->robot_no != k && !all_robot_MATpath[k].poses.empty()) {
                nav_msgs::msg::Path remaining_paths;
                for (int j = path_array_num_[k]; j < all_robot_MATpath[k].poses.size(); j++) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position = all_robot_MATpath[k].poses[j].pose.position;
                    remaining_paths.poses.push_back(pose);
                }
                route_request_msg->other_path.push_back(remaining_paths);
                if (robot_mode[k] == 1) {
                    route_request_msg->time_skip_vec.push_back(taiki_time);
                }
                else if (robot_mode[k] == 3 || robot_mode[k] == 4 || robot_mode[k] == 5) {
                    route_request_msg->time_skip_vec.push_back(-1); // 考慮しないことを意味する
                }
                else if (robot_mode[k] == 2) {
                    route_request_msg->time_skip_vec.push_back(stop_count[k]);
                }
            }
        }
    }

    // mode2かmode3,mode5のどっちに行くべきかjudge
    int mode_judge(std::vector<int>& path_plan_next, const std::vector<nav_msgs::msg::Path>& all_robot_path, int& path_plan_first, int& path_plan_now, const int& robot_no)
    {
        int path_plan_next_sum = 0;
        int void_count = 0;
        for (int j = 0; j < ROBOT_NUM; j++) {
            path_plan_next_sum += path_plan_next[j];
        }
        for (int j = 0; j < ROBOT_NUM; j++) {
            // 他のAMRは動いているか、経路計画が終わって待機しているかの混合
            if (!all_robot_path[j].poses.empty()) {
                void_count++;
            }
            // 誰か経路計画中
            // 最初用
            else if (path_plan_next_sum == (-ROBOT_NUM)) {
                path_plan_first = j;
                path_plan_now = j;
                path_plan_next[j] = robot_no;
            }
            // 最初以外用
            else {
                path_plan_now = path_plan_next[path_plan_now];
                path_plan_next[path_plan_now] = robot_no;
            }
        }
        return void_count;
    }

    // 最初のpickのとき用
    void first_pick(picks_system& picks_sys, const std::vector<std::vector<std::string>>& sub_order_pick4, const int& robot_no, std::vector<int>& order_array_num_, const std::vector<int>& str_pick_array_num_, std::vector<int>& pre_pick_left_right_retention, std::vector<int>& robot_mode, std::vector<float>& mode5_count, std::vector<int>& pre_pick_index_retention)
    {
        for (int i = 0; i < picks_sys.all_pickings.size(); i++) {
            if (picks_sys.all_pickings[i] == sub_order_pick4[robot_no][str_pick_array_num_[robot_no]]) {
                if (picks_sys.left_condition[i] == 0) {
                    // std::cout << "左行くよ(first)" << std::endl;
                    order_array_num_[robot_no] += 1;
                    picks_sys.left_condition[i] = 1;
                    pre_pick_left_right_retention[robot_no] = 0; // left
                    std::cout << "左の" << picks_sys.all_pickings[i] << "をAMR" << robot_no << "が予約" << std::endl;
                }
                else if (picks_sys.right_condition[i] == 0) {
                    // std::cout << "右行くよ(first)" << std::endl;
                    order_array_num_[robot_no] += 2;
                    picks_sys.right_condition[i] = 1;
                    pre_pick_left_right_retention[robot_no] = 1; // right
                    std::cout << "右の" << picks_sys.all_pickings[i] << "をAMR" << robot_no << "が予約" << std::endl;
                }
                else {
                    std::cout << "どっちも埋まっとるので" << picks_sys.all_pickings[i] << "が空くまでAMR" << robot_no << "は待機" << std::endl;
                    robot_mode[robot_no] = 5;
                    mode5_count[robot_no] = 0;
                    // break;
                }
                pre_pick_index_retention[robot_no] = i;
                break;
            }
        }
    }
    // pickが偶数または奇数用
    void even_odd_pick(picks_system& picks_sys, const std::vector<std::vector<std::string>>& sub_order_pick4, const int& robot_no, std::vector<int>& order_array_num_, const std::vector<int>& str_pick_array_num_, std::vector<int>& pre_pick_left_right_retention, std::vector<int>& robot_mode, std::vector<float>& mode5_count, std::vector<int>& pre_pick_index_retention, std::vector<int>& pre_pre_pick_left_right_retention, std::vector<int>& pre_pre_pick_index_retention, std::vector<bool>& odds_key)
    {
        for (int j = 0; j < picks_sys.all_pickings.size(); j++) {
            if (picks_sys.all_pickings[j] == sub_order_pick4[robot_no][str_pick_array_num_[robot_no]]) {
                pre_pre_pick_left_right_retention[robot_no] = pre_pick_left_right_retention[robot_no];
                if (order_array_num_[robot_no] % 2 == 0) { // 偶数
                    if (picks_sys.left_condition[j] == 0) {
                        // std::cout << "左行くよ(seconde,1)" << std::endl;
                        order_array_num_[robot_no] += 1;
                        picks_sys.left_condition[j] = 1;
                        pre_pick_left_right_retention[robot_no] = 0;
                        std::cout << "左の" << picks_sys.all_pickings[j] << "をAMR" << robot_no << "が予約(second,1)" << std::endl;
                    }
                    else if (picks_sys.right_condition[j] == 0) {
                        // std::cout << "右行くよ(second,1)" << std::endl;
                        order_array_num_[robot_no] += 2;
                        picks_sys.right_condition[j] = 1;
                        pre_pick_left_right_retention[robot_no] = 1;
                        std::cout << "右の" << picks_sys.all_pickings[j] << "をAMR" << robot_no << "が予約(second,1)" << std::endl;
                    }
                    else {
                        std::cout << "どっちも埋まっとるので" << picks_sys.all_pickings[j] << "が空くまでAMR" << robot_no << "は待機(second,1)" << std::endl;
                        robot_mode[robot_no] = 5;
                        mode5_count[robot_no] = 0;
                        // break;
                    }
                }
                else { // 奇数
                    if (picks_sys.left_condition[j] == 0) {
                        // std::cout << "左行くよ(second,2)" << std::endl;
                        order_array_num_[robot_no] += 2;
                        picks_sys.left_condition[j] = 1;
                        pre_pick_left_right_retention[robot_no] = 0;
                        std::cout << "左の" << picks_sys.all_pickings[j] << "をAMR" << robot_no << "が予約(second,2)" << std::endl;
                    }
                    else if (picks_sys.right_condition[j] == 0) {
                        // std::cout << "右行くよ(second,2)" << std::endl;
                        order_array_num_[robot_no] += 3;
                        picks_sys.right_condition[j] = 1;
                        pre_pick_left_right_retention[robot_no] = 1;
                        std::cout << "右の" << picks_sys.all_pickings[j] << "をAMR" << robot_no << "が予約(second,2)" << std::endl;
                    }
                    else {
                        std::cout << "どっちも埋まっとるので" << picks_sys.all_pickings[j] << "が空くまでAMR" << robot_no << "は待機(second,2)" << std::endl;
                        robot_mode[robot_no] = 5;
                        mode5_count[robot_no] = 0;
                        // break;
                    }
                }
                if (odds_key[robot_no] == true) {
                    pre_pre_pick_index_retention[robot_no] = pre_pick_index_retention[robot_no];
                    pre_pick_index_retention[robot_no] = j;
                    odds_key[robot_no] = false;
                }
                break;
            }
        }
    }
    void s_g_pick(const std::vector<std::vector<std::string>>& sub_order_pick4, const int& robot_no, std::vector<int>& order_array_num_, const std::vector<int>& str_pick_array_num_, const std::vector<int>& pre_pick_left_right_retention, const std::vector<int>& pre_pick_index_retention, std::vector<int>& pre_pre_pick_left_right_retention, std::vector<int>& pre_pre_pick_index_retention)
    {
        if (str_pick_array_num_[robot_no] == sub_order_pick4[robot_no].size()) {
            pre_pre_pick_left_right_retention[robot_no] = pre_pick_left_right_retention[robot_no];
            pre_pre_pick_index_retention[robot_no] = pre_pick_index_retention[robot_no];
        }
        if (order_array_num_[robot_no] == sub_order_pick4[robot_no].size() * 2 - 1) { // 4つのpick×2-1
            order_array_num_[robot_no] += 2;
        }
        else {
            order_array_num_[robot_no] += 1;
        }
    }

public:
    RouteController(const rclcpp::NodeOptions& options) : RouteController("", options) {}
    RouteController(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_controller_node", name_space, options) //, all_robot_path(ROBOT_NUM)
    {
        using namespace std::chrono_literals;

        defaultPath.poses.resize(2);
        geometry_msgs::msg::PoseStamped pose1;
        pose1.pose.position.x = 1.0;
        pose1.pose.position.y = 1.0;
        defaultPath.poses[0] = pose1;

        geometry_msgs::msg::PoseStamped pose2;
        pose2.pose.position.x = 1.0;
        pose2.pose.position.y = 1.0;
        defaultPath.poses[1] = pose2;
        all_robot_path = std::vector<nav_msgs::msg::Path>(ROBOT_NUM, defaultPath);
        all_robot_MATpath = std::vector<nav_msgs::msg::Path>(ROBOT_NUM, defaultPath);

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_topic_test", rclcpp::QoS(10));
        route_request_publisher_ = this->create_publisher<my_msgs::msg::RouteRequest>("route_request_topic_test", rclcpp::QoS(10));
        re_order_publisher_ = this->create_publisher<my_msgs::msg::ReOrder>("reorder", rclcpp::QoS(10));
        static auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.frame_id = "map"; // 基準座標を決めている、送るときだけ

        // 経路再生成をサブスクライブ
        re_route_request_subscription = this->create_subscription<my_msgs::msg::ReRouteRequest>("re_route_request_topic_test", rclcpp::QoS(10), [&](const my_msgs::msg::ReRouteRequest::SharedPtr re_route_request_msg) {
            // std::cout << "AMR" << re_route_request_msg->robot_no << "はmode2での滞在時間: " << stop_count[re_route_request_msg->robot_no] << "秒" << std::endl;
            mode_condition[re_route_request_msg->robot_no].push_back(2);
            mode_condition_time[re_route_request_msg->robot_no].push_back(stop_count[re_route_request_msg->robot_no]);
            stop_count[re_route_request_msg->robot_no] = 0;
            auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
            route_request_msg->robot_no = re_route_request_msg->robot_no;
            route_request_msg->other_path_no = -1; ////////////////////////////////////
            route_request_msg->start_num = re_route_request_msg->start_num;
            route_request_msg->start.pose.position.x = re_route_request_msg->start.pose.position.x;
            route_request_msg->start.pose.position.y = re_route_request_msg->start.pose.position.y;
            route_request_msg->goal_num = re_route_request_msg->goal_num;
            route_request_msg->goal.pose.position.x = re_route_request_msg->goal.pose.position.x;
            route_request_msg->goal.pose.position.y = re_route_request_msg->goal.pose.position.y;

            create_time_skip_vec(route_request_msg, all_robot_MATpath, path_array_num_, robot_mode, stop_count);

            std::cout << "AMR" << route_request_msg->robot_no << "再経路計画start" << std::endl;
            route_request_publisher_->publish(*route_request_msg);
        });

        // 全pick情報をサブスクライブ
        picks_subscription = this->create_subscription<my_msgs::msg::Picks>("picks", rclcpp::QoS(10), [&](const my_msgs::msg::Picks::SharedPtr picks_msg) {
            picks_sys.all_pickings = picks_msg->all_picks;
            // for (int i = 0; i < picks_sys.all_pickings.size(); i++) {
            //     std::cout << picks_sys.all_pickings[i] << std::endl;
            // }
            picks_sys.left_condition.assign(picks_sys.all_pickings.size(), 0);
            picks_sys.right_condition.assign(picks_sys.all_pickings.size(), 0);
        });

        // オーダーのパスをサブスクライブ
        order_request_subscription = this->create_subscription<my_msgs::msg::OrderRequest>("order_request_topic_test", rclcpp::QoS(10), [&](const my_msgs::msg::OrderRequest::SharedPtr order_request_msg) {
            std::cout << "AMR" << order_request_msg->robot_no << " " << order_100_count << " 個目";
            for (int i = 0; i < order_request_msg->pick_4.size(); i++) {
                std::cout << order_request_msg->pick_4[i] << " ";
            }
            std::cout << std::endl;

            order_100_count += 1;

            sub_order_paths[order_request_msg->robot_no].poses.clear();
            order_array_num_[order_request_msg->robot_no] = 0;
            str_pick_array_num_[order_request_msg->robot_no] = 0;
            path_array_num_[order_request_msg->robot_no] = 0;
            // pre_pre_pick_index_retention = std::vector<int>(ROBOT_NUM, -1);
            // pre_pre_pick_left_right_retention = std::vector<int>(ROBOT_NUM, -1);
            // 座標を格納用
            for (int i = 0; i < order_request_msg->order_paths.poses.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = order_request_msg->order_paths.poses[i].pose.position.x;
                pose.pose.position.y = order_request_msg->order_paths.poses[i].pose.position.y;
                pose.pose.position.z = order_request_msg->order_paths.poses[i].pose.position.z;
                sub_order_paths[order_request_msg->robot_no].poses.push_back(pose);
            }
            // string情報（例えば、A0とかを格納用）
            for (int i = 0; i < order_request_msg->pick_4.size(); i++) {
                sub_order_pick4[order_request_msg->robot_no][i] = order_request_msg->pick_4[i];
            }

            // mode2かmode3,mode5のどっちに行くべきかjudge
            int void_count = mode_judge(path_plan_next, all_robot_path, path_plan_first, path_plan_now, order_request_msg->robot_no);

            // std::cout << "AMR" << order_request_msg->robot_no << "はmode4での滞在時間: " << mode4_count[order_request_msg->robot_no] << "秒" << std::endl;
            // mode_condition[order_request_msg->robot_no].push_back(4);
            // mode_condition_time[order_request_msg->robot_no].push_back(mode4_count[order_request_msg->robot_no]);
            // ここでmode3かmode2かのjudge
            if (void_count != ROBOT_NUM) { // mode3
                // std::cout << "AMR" << order_request_msg->robot_no << "はmode3へ移行 " << std::endl;
                robot_mode[order_request_msg->robot_no] = 3;
                mode3_count[order_request_msg->robot_no] = 0;
            }
            else { // mode2だが、mode5に行く可能性も
                robot_mode[order_request_msg->robot_no] = 2;
                // 最初のroute_request.msgを送信
                auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
                route_request_msg->robot_no = order_request_msg->robot_no;
                route_request_msg->other_path_no = -1; ////////////////////////////////////
                route_request_msg->start_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;
                route_request_msg->start.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                route_request_msg->start.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                // 次目的地がpick地点なら
                if (str_pick_array_num_[route_request_msg->robot_no] < sub_order_pick4[route_request_msg->robot_no].size()) {
                    first_pick(picks_sys, sub_order_pick4, route_request_msg->robot_no, order_array_num_, str_pick_array_num_, pre_pick_left_right_retention, robot_mode, mode5_count, pre_pick_index_retention);
                }
                if (robot_mode[order_request_msg->robot_no] == 2) {
                    route_request_msg->goal.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                    route_request_msg->goal.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                    route_request_msg->goal_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;

                    create_time_skip_vec(route_request_msg, all_robot_MATpath, path_array_num_, robot_mode, stop_count);

                    // str_pick_array_num_[route_request_msg->robot_no] += 1;
                    all_robot_path[route_request_msg->robot_no].poses.clear();
                    std::cout << "AMR" << route_request_msg->robot_no << "経路計画start" << std::endl;
                    route_request_publisher_->publish(*route_request_msg);
                }
            }
        });

        // 該当するロボットのcalc_pathを受信
        for (size_t n = 0; n < ROBOT_NUM; n++) {
            calc_path_subscription_.push_back(this->create_subscription<nav_msgs::msg::Path>("calc_path_" + std::to_string(n), rclcpp::QoS(10), [&, n](const nav_msgs::msg::Path::SharedPtr msg) { all_robot_path[n] = *msg; }));
        }
        // 該当するロボットのcalc_MATpathを受信
        for (size_t n = 0; n < ROBOT_NUM; n++) {
            calc_MATpath_subscription_.push_back(this->create_subscription<nav_msgs::msg::Path>("calc_MATpath_" + std::to_string(n), rclcpp::QoS(10), [&, n](const nav_msgs::msg::Path::SharedPtr msg) { all_robot_MATpath[n] = *msg; }));
        }

        // 各pose の送受信のために名前決め
        for (size_t i = 0; i < ROBOT_NUM; i++) {
            vec_pose_publisher_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_" + std::to_string(i), rclcpp::QoS(10)));
        }

        point_timer_ = this->create_wall_timer(10ms, [&]() {
            // std::cout << conflict_count << std::endl;

            conf_count++;
            if (conf_count == 200) {
                conf_count = 0;
                std::cout << "conflict_count:" << conflict_count << std::endl;
            }

            past_pairs = current_pairs;
            current_pairs.clear();

            for (size_t i = 0; i < current_move_pose.size(); ++i) {
                for (size_t j = i + 1; j < current_move_pose.size(); ++j) {
                    float distance = calculateDistance(current_move_pose[i], current_move_pose[j]);
                    if (distance < 0.5) {
                        current_pairs.emplace_back(i, j); // ここで emplace_back を使用
                    }
                }
            }

            int pair_accordance = 0;

            for (const auto& curr_pair : current_pairs) {
                for (const auto& pas_pair : past_pairs) {
                    if (curr_pair == pas_pair) {
                        pair_accordance++;
                    }
                }
            }

            if (past_pairs.size() - pair_accordance) {
                conflict_count++;
            }

            // std::cout << "conflict_count:" << conflict_count << std::endl;

            for (int i = 0; i < ROBOT_NUM; i++) {
                auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
                // void_count = 0;
                pose_msg->header.frame_id = "map";
                // ピックロケに到着したあとの10秒間待機用
                if (robot_mode[i] == 2) {
                    if (stop_count[i] == 0) {
                        // std::cout << "AMR" << i << "はmode2始まるよ " << std::endl;
                    }
                    if (!all_robot_path[i].poses.empty() && stop_count[i] > taiki_time) {
                        robot_mode[i] = 1;
                        current_pose[i] = {all_robot_path[i].poses[path_array_num_[i]].pose.position.x, all_robot_path[i].poses[path_array_num_[i]].pose.position.y};
                        next_pose[i] = {all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.x, all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.y};
                        current_move_pose[i] = current_pose[i];
                        path_array_num_[i] += 1;
                        std::cout << "AMR" << i << "はmode2での滞在時間: " << stop_count[i] << "秒" << std::endl;
                        mode_condition[i].push_back(2);
                        mode_condition_time[i].push_back(stop_count[i]);
                        stop_count[i] = 0;
                        // std::cout << "AMR" << i << "はmode1へ移行 " << std::endl;
                        odds_key[i] = true;
                        mode1_count[i] = 0;
                        if (str_pick_array_num_[i] > 0 && str_pick_array_num_[i] < sub_order_pick4[i].size() + 1) {
                            if (pre_pre_pick_left_right_retention[i] == 0) { // 左解除
                                picks_sys.left_condition[pre_pre_pick_index_retention[i]] = 0;
                                std::cout << "左の" << picks_sys.all_pickings[pre_pre_pick_index_retention[i]] << "をAMR" << i << "が解除" << std::endl;
                            }
                            else if (pre_pre_pick_left_right_retention[i] == 1) { // 右解除
                                picks_sys.right_condition[pre_pre_pick_index_retention[i]] = 0;
                                std::cout << "右の" << picks_sys.all_pickings[pre_pre_pick_index_retention[i]] << "をAMR" << i << "が解除" << std::endl;
                            }
                            else {
                                std::cout << "errrorrrr" << std::endl;
                            }
                        }
                        str_pick_array_num_[i] += 1;
                    }
                    else {
                        stop_count[i] += 0.01;
                        pose_msg->pose.position.x = current_move_pose[i][0];
                        pose_msg->pose.position.y = current_move_pose[i][1];
                        vec_pose_publisher_[i]->publish(*pose_msg);
                        continue;
                    }
                }
                // 他AMRのいずれかが経路生成中のときの待機用
                if (robot_mode[i] == 3) {
                    pose_msg->pose.position.x = current_move_pose[i][0];
                    pose_msg->pose.position.y = current_move_pose[i][1];
                    vec_pose_publisher_[i]->publish(*pose_msg);
                    mode3_count[i] += 0.01;
                    int void_count = 0;
                    for (int j = 0; j < ROBOT_NUM; j++) {
                        if (!all_robot_path[j].poses.empty()) {
                            void_count++;
                        }
                    }
                    if (void_count == ROBOT_NUM && i == path_plan_next[path_plan_first]) {
                        auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
                        path_plan_next[path_plan_first] = -1;
                        route_request_msg->other_path_no = path_plan_first;
                        path_plan_first = i;
                        // mode2だが、mode5に行く可能性あり
                        robot_mode[i] = 2;
                        route_request_msg->robot_no = i;
                        route_request_msg->start_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;
                        route_request_msg->start.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                        route_request_msg->start.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                        std::cout << "AMR" << i << "はmode3での滞在時間: " << mode3_count[i] << "秒" << std::endl;
                        mode_condition[i].push_back(3);
                        mode_condition_time[i].push_back(mode3_count[i]);
                        // 次目的地がpick地点なら
                        if (str_pick_array_num_[route_request_msg->robot_no] == 0) { // 初pick
                            first_pick(picks_sys, sub_order_pick4, route_request_msg->robot_no, order_array_num_, str_pick_array_num_, pre_pick_left_right_retention, robot_mode, mode5_count, pre_pick_index_retention);
                        }
                        else if (str_pick_array_num_[route_request_msg->robot_no] < sub_order_pick4[route_request_msg->robot_no].size()) {
                            even_odd_pick(picks_sys, sub_order_pick4, route_request_msg->robot_no, order_array_num_, str_pick_array_num_, pre_pick_left_right_retention, robot_mode, mode5_count, pre_pick_index_retention, pre_pre_pick_left_right_retention, pre_pre_pick_index_retention, odds_key);
                        }
                        else {
                            s_g_pick(sub_order_pick4, route_request_msg->robot_no, order_array_num_, str_pick_array_num_, pre_pick_left_right_retention, pre_pick_index_retention, pre_pre_pick_left_right_retention, pre_pre_pick_index_retention);
                        }

                        if (robot_mode[route_request_msg->robot_no] == 2) {

                            route_request_msg->goal.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                            route_request_msg->goal.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                            route_request_msg->goal_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;

                            create_time_skip_vec(route_request_msg, all_robot_MATpath, path_array_num_, robot_mode, stop_count);
                            all_robot_path[i].poses.clear();

                            std::cout << "AMR" << route_request_msg->robot_no << "経路計画start" << std::endl;
                            route_request_publisher_->publish(*route_request_msg);
                        }
                    }

                    continue;
                }
                // goal用
                if (robot_mode[i] == 4) {
                    // mode4_count[i] += 0.01;
                    pose_msg->pose.position.x = current_move_pose[i][0];
                    pose_msg->pose.position.y = current_move_pose[i][1];
                    vec_pose_publisher_[i]->publish(*pose_msg);

                    continue;
                }
                if (robot_mode[i] == 5) {
                    pose_msg->pose.position.x = current_move_pose[i][0];
                    pose_msg->pose.position.y = current_move_pose[i][1];
                    vec_pose_publisher_[i]->publish(*pose_msg);

                    mode5_count[i] += 0.01;

                    if (picks_sys.left_condition[pre_pick_index_retention[i]] == 0) {
                        // std::cout << "mode5終わり" << std::endl;
                        std::cout << "左の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "解除された" << std::endl;
                        std::cout << "AMR" << i << "はmode5での滞在時間: " << mode5_count[i] << "秒" << std::endl;
                        mode_condition[i].push_back(5);
                        mode_condition_time[i].push_back(mode5_count[i]);
                        // mode2かmode3,mode5のどっちに行くべきかjudge
                        int void_count = mode_judge(path_plan_next, all_robot_path, path_plan_first, path_plan_now, i);
                        // ここでmode3かmode2かのjudge
                        if (void_count != ROBOT_NUM) { // mode3
                            // std::cout << "AMR" << i << "はmode3へ移行 " << std::endl;
                            robot_mode[i] = 3;
                            mode3_count[i] = 0;
                        }
                        else { // mode2
                            robot_mode[i] = 2;
                            auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
                            route_request_msg->robot_no = i;
                            route_request_msg->other_path_no = -1; ////////////////////////////////////
                            route_request_msg->start_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;
                            route_request_msg->start.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                            route_request_msg->start.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                            if (str_pick_array_num_[route_request_msg->robot_no] == 0) {
                                // std::cout << "左行くよ(mode5,1)" << std::endl;
                                order_array_num_[route_request_msg->robot_no] += 1;
                                picks_sys.left_condition[pre_pick_index_retention[i]] = 1;
                                pre_pick_left_right_retention[route_request_msg->robot_no] = 0;
                                std::cout << "左の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "をAMR" << i << "が予約(3,1)" << std::endl;
                            }
                            else if (str_pick_array_num_[route_request_msg->robot_no] < sub_order_pick4[route_request_msg->robot_no].size()) {

                                if (order_array_num_[route_request_msg->robot_no] % 2 == 0) { // 偶数
                                    // std::cout << "左行くよ(mode5,2)" << std::endl;
                                    order_array_num_[route_request_msg->robot_no] += 1;
                                    picks_sys.left_condition[pre_pick_index_retention[i]] = 1;
                                    pre_pick_left_right_retention[route_request_msg->robot_no] = 0;
                                    std::cout << "左の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "をAMR" << i << "が予約(3,2)" << std::endl;
                                }
                                else { // 奇数
                                    // std::cout << "左行くよ(mode5,3)" << std::endl;
                                    order_array_num_[route_request_msg->robot_no] += 2;
                                    picks_sys.left_condition[pre_pick_index_retention[i]] = 1;
                                    pre_pick_left_right_retention[route_request_msg->robot_no] = 0;
                                    std::cout << "左の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "をAMR" << i << "が予約(3,3)" << std::endl;
                                }
                            }
                            else {
                                std::cout << "arienaierrorrr" << std::endl;
                            }
                            // pre_pre_pick_index_retention[i] = pre_pick_index_retention[i];
                            // pre_pick_index_retention[robot_no] = j;
                            route_request_msg->goal.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                            route_request_msg->goal.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                            route_request_msg->goal_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;

                            create_time_skip_vec(route_request_msg, all_robot_MATpath, path_array_num_, robot_mode, stop_count);
                            all_robot_path[i].poses.clear();
                            route_request_publisher_->publish(*route_request_msg);
                        }
                    }
                    else if (picks_sys.right_condition[pre_pick_index_retention[i]] == 0) {
                        // std::cout << "mode5終わり" << std::endl;
                        std::cout << "右の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "解除された" << std::endl;
                        std::cout << "AMR" << i << "はmode5での滞在時間: " << mode5_count[i] << "秒" << std::endl;
                        mode_condition[i].push_back(5);
                        mode_condition_time[i].push_back(mode5_count[i]);
                        // mode2かmode3,mode5のどっちに行くべきかjudge
                        int void_count = mode_judge(path_plan_next, all_robot_path, path_plan_first, path_plan_now, i);
                        // ここでmode3かmode2かのjudge
                        if (void_count != ROBOT_NUM) { // mode3
                            // std::cout << "AMR" << i << "はmode3へ移行 " << std::endl;
                            robot_mode[i] = 3;
                            mode3_count[i] = 0;
                        }
                        else { // mode2
                            robot_mode[i] = 2;
                            auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
                            route_request_msg->robot_no = i;
                            route_request_msg->other_path_no = -1; ////////////////////////////////////
                            route_request_msg->start_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;
                            route_request_msg->start.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                            route_request_msg->start.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                            if (str_pick_array_num_[route_request_msg->robot_no] == 0) {
                                // std::cout << "右行くよ(mode5,1)" << std::endl;
                                order_array_num_[route_request_msg->robot_no] += 2;
                                picks_sys.right_condition[pre_pick_index_retention[i]] = 1;
                                pre_pick_left_right_retention[route_request_msg->robot_no] = 1;
                                std::cout << "右の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "をAMR" << i << "が予約(3,1)" << std::endl;
                            }
                            else if (str_pick_array_num_[route_request_msg->robot_no] < sub_order_pick4[route_request_msg->robot_no].size()) {
                                if (order_array_num_[route_request_msg->robot_no] % 2 == 0) { // 偶数
                                    // std::cout << "右行くよ(mode5,2)" << std::endl;
                                    order_array_num_[route_request_msg->robot_no] += 2;
                                    picks_sys.right_condition[pre_pick_index_retention[i]] = 1;
                                    pre_pick_left_right_retention[route_request_msg->robot_no] = 1;
                                    std::cout << "右の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "をAMR" << i << "が予約(3,2)" << std::endl;
                                }
                                else {
                                    // std::cout << "右行くよ(mode5,3)" << std::endl;
                                    order_array_num_[route_request_msg->robot_no] += 3;
                                    picks_sys.right_condition[pre_pick_index_retention[i]] = 1;
                                    pre_pick_left_right_retention[route_request_msg->robot_no] = 1;
                                    std::cout << "右の" << picks_sys.all_pickings[pre_pick_index_retention[i]] << "をAMR" << i << "が予約(3,3)" << std::endl;
                                }
                            }
                            else {
                                std::cout << "arienaierrorrr" << std::endl;
                            }
                            route_request_msg->goal.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                            route_request_msg->goal.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                            route_request_msg->goal_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;

                            create_time_skip_vec(route_request_msg, all_robot_MATpath, path_array_num_, robot_mode, stop_count);
                            all_robot_path[i].poses.clear();
                            route_request_publisher_->publish(*route_request_msg);
                        }
                    }

                    continue;
                }
                if (robot_mode[i] == 1) {
                    // std::cout << next_pose[i][0] - current_move_pose[i][0] << " " << next_pose[i][1] - current_move_pose[i][1] << std::endl;
                    if (std::sqrt(std::pow(next_pose[i][0] - current_move_pose[i][0], 2) + std::pow(next_pose[i][1] - current_move_pose[i][1], 2)) < v * 0.01) {
                        mode1_count[i] += 0.01;
                        // ピックロケ到着時、モードチェンジ
                        if (next_pose[i][0] == all_robot_path[i].poses.back().pose.position.x && next_pose[i][1] == all_robot_path[i].poses.back().pose.position.y) {
                            robot_mode[i] = 2;
                            path_array_num_[i] = 0;
                            std::cout << "AMR" << i << "はmode1での移動時間: " << mode1_count[i] << "秒" << std::endl;
                            mode_condition[i].push_back(1);
                            mode_condition_time[i].push_back(mode1_count[i]);
                            mode1_count[i] = 0;

                            // order完遂時
                            if (next_pose[i][0] == sub_order_paths[i].poses.back().pose.position.x && next_pose[i][1] == sub_order_paths[i].poses.back().pose.position.y) {
                                robot_mode[i] = 4;
                                mode4_count[i] = 0;
                                if (order_100_count == 100) {
                                    csv_one_count++;
                                    if (csv_one_count == ROBOT_NUM) {

                                        writeCSV_A(condition_csv_path, mode_condition, "mode_condition_AMR" + std::to_string(ROBOT_NUM) + '_');
                                        writeCSV_A(condition_time_csv_path, mode_condition_time, "mode_condition_time_AMR" + std::to_string(ROBOT_NUM) + '_');
                                        std::cout << conflict_count << std::endl;
                                    }
                                }
                                if (order_100_count < 100) {
                                    auto re_order_msg = std::make_shared<my_msgs::msg::ReOrder>();
                                    re_order_msg->robot_no = i;
                                    re_order_msg->order_no = order_100_count;
                                    re_order_publisher_->publish(*re_order_msg);
                                }
                            }
                            else {
                                // ピックロケがorder完遂地点じゃない場合、経路生成、ただし、他AMRのいずれかが経路生成中はまだルートリクエストメッセージは送らず保留
                                int void_count = mode_judge(path_plan_next, all_robot_path, path_plan_first, path_plan_now, i);
                                // ここでmode3かmode2かのjudge
                                if (void_count != ROBOT_NUM) {
                                    // std::cout << "AMR" << i << "はmode3へ移行 " << std::endl;
                                    robot_mode[i] = 3;
                                    mode3_count[i] = 0;
                                }
                                else {
                                    robot_mode[i] = 2;
                                    auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
                                    route_request_msg->robot_no = i;
                                    route_request_msg->other_path_no = -1; // 経路計画してるやつがゼロであることを意味する

                                    route_request_msg->start_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;
                                    route_request_msg->start.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                                    route_request_msg->start.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                                    // 次目的地がpick地点なら
                                    if (str_pick_array_num_[route_request_msg->robot_no] < sub_order_pick4[route_request_msg->robot_no].size()) {
                                        even_odd_pick(picks_sys, sub_order_pick4, route_request_msg->robot_no, order_array_num_, str_pick_array_num_, pre_pick_left_right_retention, robot_mode, mode5_count, pre_pick_index_retention, pre_pre_pick_left_right_retention, pre_pre_pick_index_retention, odds_key);
                                    }
                                    else {
                                        s_g_pick(sub_order_pick4, route_request_msg->robot_no, order_array_num_, str_pick_array_num_, pre_pick_left_right_retention, pre_pick_index_retention, pre_pre_pick_left_right_retention, pre_pre_pick_index_retention);
                                    }

                                    if (robot_mode[route_request_msg->robot_no] == 2) {

                                        route_request_msg->goal.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
                                        route_request_msg->goal.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
                                        route_request_msg->goal_num = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.z;

                                        create_time_skip_vec(route_request_msg, all_robot_MATpath, path_array_num_, robot_mode, stop_count);
                                        // str_pick_array_num_[route_request_msg->robot_no] += 1;
                                        std::cout << "AMR" << route_request_msg->robot_no << "経路計画start" << std::endl;
                                        all_robot_path[i].poses.clear();
                                        route_request_publisher_->publish(*route_request_msg);
                                    }
                                }
                            }
                        }
                        current_move_pose[i][0] = next_pose[i][0];
                        current_move_pose[i][1] = next_pose[i][1];
                        pose_msg->pose.position.x = current_move_pose[i][0];
                        pose_msg->pose.position.y = current_move_pose[i][1];
                        vec_pose_publisher_[i]->publish(*pose_msg);
                        current_pose[i] = next_pose[i];
                        next_pose[i] = {all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.x, all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.y};
                        path_array_num_[i] += 1;
                    }
                    else {
                        diff_pose[i] = {next_pose[i][0] - current_pose[i][0], next_pose[i][1] - current_pose[i][1]};
                        if (diff_pose[i][0] != 0 && diff_pose[i][1] != 0) {
                            next_move_pose[i] = {current_move_pose[i][0] + inv_route_2 * v * 0.1 * diff_pose[i][0], current_move_pose[i][1] + inv_route_2 * v * 0.1 * diff_pose[i][1]};
                        }
                        else {
                            next_move_pose[i] = {current_move_pose[i][0] + v * 0.1 * diff_pose[i][0], current_move_pose[i][1] + v * 0.1 * diff_pose[i][1]};
                        }
                        current_move_pose[i] = next_move_pose[i];
                        pose_msg->pose.position.x = current_move_pose[i][0];
                        pose_msg->pose.position.y = current_move_pose[i][1];
                        vec_pose_publisher_[i]->publish(*pose_msg);
                        mode1_count[i] += 0.01;
                    }
                }
            }
        });
    }
};

} // namespace tlab

// kokokara///////////////////////////////////////////////////////////////////////////////////////////////
//  auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
//                  pose_msg->header.frame_id = "map";
//                  // std::cout << path_attentive_last_pose[i][0] << " " << path_attentive_last_pose[i][1] << std::endl;
//                  if (all_robot_path[i].poses.empty() && path_attentive_last_pose[i][0] == 0 && path_attentive_last_pose[i][1] == 0) {

//                     // std::cout << "cccccccccccccccc" << std::endl;
//                     // std::cout << " void_test " << std::endl;
//                 }
//                 else {path_plan_firstve_last_pose[i][0] == 0 && path_attentive_last_pose[i][1] == 0))) {
//                         // picking場所到達後の処理
//                         pose_msg->pose.position.x = path_attentive_last_pose[i][0];
//                         pose_msg->pose.position.y = path_attentive_last_pose[i][1];
//                         // std::cout << "b" << std::endl;

//                         if (picking_condition[i]) {

//                             auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
//                             route_request_msg->robot_no = i;
//                             route_request_msg->start.pose.position = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position;
//                             route_request_msg->goal.pose.position = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no] + 1].pose.position;
//                             for (int i = 0; i < ROBOT_NUM; i++) {
//                                 if (route_request_msg->robot_no != i && !all_robot_path[i].poses.empty()) {
//                                     nav_msgs::msg::Path remaining_paths;
//                                     for (int j = path_array_num_[i]; j < all_robot_path[i].poses.size(); j++) {
//                                         geometry_msgs::msg::PoseStamped pose;
//                                         pose.pose.position = all_robot_path[i].poses[j].pose.position;
//                                         remaining_paths.poses.push_back(pose);
//                                     }
//                                     route_request_msg->other_path.push_back(remaining_paths);
//                                 }
//                             }
//                             order_array_num_[route_request_msg->robot_no] += 1;

//                             picking_condition[i] = false;
//                             route_request_publisher_->publish(*route_request_msg);
//                         }
//                     }
//                     else {
//                         if ((all_robot_path[i].poses.size() - 1) < path_array_num_[i]) {
//                             // std::cout << "dddd" << std::endl;
//                             // auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
//                             // pose_msg->header.frame_id = "map";
//                             path_array_num_[i] = 0;
//                             if (order_array_num_[i] < sub_order_paths[i].poses.size() - 1) {
//                                 picking_condition[i] = true;
//                             }
//                             else {
//                                 picking_condition[i] = false;
//                             }

//                             pose_msg->pose.position.x = path_attentive_last_pose[i][0];
//                             pose_msg->pose.position.y = path_attentive_last_pose[i][1];
//                             all_robot_path[i].poses.clear();
//                             // vec_pose_publisher_[i]->publish(*pose_msg);
//                             // }
//                         }
//                         else {
//                             // std::cout << "aaaaaa" << std::endl;
//                             float distance = std::sqrt(std::pow(now_pose[0] - move_pose_before[0], 2) + std::pow(now_pose[1] - move_pose_before[1], 2));
//                             if (distance < v * 0.1 * 0.75) {
//                                 path_array_num_[i] += 1;
//                                 path_attentive_last_pose[i][0] = now_pose[0];
//                                 path_attentive_last_pose[i][0] = now_pose[1];
//                             }
//                             auto p1 = std::make_shared<geometry_msgs::msg::PoseStamped>();
//                             auto p2 = std::make_shared<geometry_msgs::msg::PoseStamped>();
//                             p1 = std::make_shared<geometry_msgs::msg::PoseStamped>(all_robot_path[i].poses[path_array_num_[i]]);
//                             p2 = std::make_shared<geometry_msgs::msg::PoseStamped>(all_robot_path[i].poses[path_array_num_[i] + 1]);
//                             last_pose = {p1->pose.position.x, p1->pose.position.y};
//                             now_pose = {p2->pose.position.x, p2->pose.position.y};
//                             diff_pose = {now_pose[0] - last_pose[0], now_pose[1] - last_pose[1]};

//                             if (diff_pose[0] + diff_pose[1] != 0) {
//                                 dx = inv_route_2 * v * 0.1 * diff_pose[0];
//                                 dy = inv_route_2 * v * 0.1 * diff_pose[1];
//                             }
//                             else {
//                                 dx = v * 0.1 * diff_pose[0];
//                                 dy = v * 0.1 * diff_pose[1];
//                             }
//                             move_pose_next = {move_pose_before[0] + dx, move_pose_before[1] + dy};
//                             move_pose_before = move_pose_next;

//                             pose_msg->pose.position.x = move_pose_before[0];
//                             pose_msg->pose.position.y = move_pose_before[1];

//                             // pose_msg->pose.position.x = p1->pose.position.x;
//                             // pose_msg->pose.position.y = p1->pose.position.y;
//                             // path_array_num_[i] += 1;

//                             // path_attentive_last_pose[i][0] = pose_msg->pose.position.x;
//                             // path_attentive_last_pose[i][1] = pose_msg->pose.position.y;
//                         }
//                         // vec_pose_publisher_[i]->publish(*pose_msg);
//                     }
//                     // std::cout << pose_msg->pose.position.x << " " << pose_msg->pose.position.y << std::endl;
//                     vec_pose_publisher_[i]->publish(*pose_msg);
//                 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
// publisher_ = this->create_publisher<std_msgs::msg::String>("topic_test",rclcpp::QoS(10));
// path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_topic_test", rclcpp::QoS(10));
// RCLCPP_INFO(this->get_logger(), "calc_path_sub : %d", i);
// timer_ = this->create_wall_timer(1000ms, [&]() {
// auto msg = std::make_shared<std_msgs::msg::String>();
// msg->data = "Hello " + std::to_string(count_++);
// RCLCPP_INFO(this->get_logger(), "Pub:%s", msg->data.c_str());
// publisher_->publish(*msg);

// auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
// pose_msg->header.frame_id = "map";
// pose_msg->pose.position.x = 11.1;
// RCLCPP_INFO(this->get_logger(), "Pub:%lf", pose_msg->pose.position.x);
// pose_publisher_->publish(*pose_msg);

// geometry_msgs::msg::PoseStamped p;
// p.pose.position.x = 20.55 + x_value_;
// x_value_ += 1.0;
// path_msg->poses.push_back(p);

// // RCLCPP_INFO(this->get_logger(), "Pub:%lf", path_msg->poses.back().pose.position.x);
// path_publisher_->publish(*path_msg);
//}

// if (!all_robot_path[1].poses.empty() && all_robot_path[2].poses.empty()) {
//     std::cout << " love_msg " << std::endl;
//     {
//         // rclcpp::sleep_for(std::chrono::seconds(5));
//         int send_no = 2;
//         auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
//         route_request_msg->robot_no = send_no;
//         route_request_msg->start.pose.position.x = 4.0;
//         route_request_msg->start.pose.position.y = 2.0;
//         route_request_msg->goal.pose.position.x = 4.0;
//         route_request_msg->goal.pose.position.y = 11.0;
//         for (int i = 0; i < all_robot_path.size(); i++) {
//             if (send_no != i && !all_robot_path[i].poses.empty()) {

//                 std::cout << all_robot_path[i].poses.size() << std::endl;
//                 std::cout << " robot1_path_size " << std::endl;
//                 std::cout << path_array_num_[i] << std::endl;

//                 nav_msgs::msg::Path remaining_paths;

//                 for (int j = path_array_num_[i]; j < all_robot_path[i].poses.size(); j++) {
//                     geometry_msgs::msg::PoseStamped pose;
//                     pose.pose.position.x = all_robot_path[i].poses[j].pose.position.x;
//                     pose.pose.position.y = all_robot_path[i].poses[j].pose.position.y;
//                     remaining_paths.poses.push_back(pose);
//                 }

//                 route_request_msg->other_path.push_back(remaining_paths);
//             }
//         }
//         // rclcpp::sleep_for(std::chrono::seconds(5));
//         std::cout << " test_pub2_route_request_msg " << std::endl;
//         route_request_publisher_->publish(*route_request_msg);
//     }
// }

// int send_no = 1;
//             auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
//             route_request_msg->robot_no = send_no;
//             route_request_msg->start.pose.position.x = 2.0;
//             route_request_msg->start.pose.position.y = 2.0;
//             route_request_msg->goal.pose.position.x = 12.0;
//             route_request_msg->goal.pose.position.y = 9.0;
//             for (int i = 0; i < all_robot_path.size(); i++) {
//                 if (send_no != i && !all_robot_path[i].poses.empty()) {

//                     nav_msgs::msg::Path remaining_paths;

//                     for (int j = path_array_num_[i]; j < all_robot_path[i].poses.size(); j++) {
//                         geometry_msgs::msg::PoseStamped pose;
//                         pose.pose.position.x = all_robot_path[i].poses[j].pose.position.x;
//                         pose.pose.position.y = all_robot_path[i].poses[j].pose.position.y;
//                         remaining_paths.poses.push_back(pose);
//                     }

//                     route_request_msg->other_path.push_back(remaining_paths);
//                 }
//             }
//             rclcpp::sleep_for(std::chrono::seconds(5));
//             std::cout << " test_pub1_route_request_msg " << std::endl;
//             route_request_publisher_->publish(*route_request_msg);

// std::cout << " calcp_path_test " << std::endl;
// attentive_path_subscription = this->create_subscription<nav_msgs::msg::Path>("calc_path_1", rclcpp::QoS(10), [&](const nav_msgs::msg::Path::SharedPtr msg_test) {

// else if (all_robot_path[i].poses.empty() && (!path_attentive_last_pose.empty()))
//             {
//                 std::cout << "bb" << std::endl;
//                 pose_msg->pose.position.x = path_attentive_last_pose[0];
//                 pose_msg->pose.position.y = path_attentive_last_pose[1];
//                 vec_pose_publisher_[i]->publish(*pose_msg);
//             }
//             else
//             {
//                 std::cout << "aaaaaaaaaaaaaaaaaaaa" << std::endl;
//                 if (all_robot_path[i].poses.size() < path_array_num_[i]) {
//                     // auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
//                     // pose_msg->header.frame_id = "map";
//                     path_array_num_[i] = 0;
//                     pose_msg->pose.position.x = path_attentive_last_pose[0];
//                     pose_msg->pose.position.y = path_attentive_last_pose[1];
//                     all_robot_path[i].poses.clear();
//                     // vec_pose_publisher_[i]->publish(*pose_msg);
//                     // }
//                 }
//                 else {
//                     auto p1 = std::make_shared<geometry_msgs::msg::PoseStamped>();
//                     // auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
//                     // pose_msg->header.frame_id = "map";
//                     // p1 = std::make_shared<geometry_msgs::msg::PoseStamped>(msg->poses[path_array_num_]);
//                     p1 = std::make_shared<geometry_msgs::msg::PoseStamped>(all_robot_path[i].poses[path_array_num_[i]]);
//                     pose_msg->pose.position.x = p1->pose.position.x;
//                     pose_msg->pose.position.y = p1->pose.position.y;
//                     // RCLCPP_INFO(this->get_logger(), "Pub:%lf", pose_msg->pose.position.x);
//                     // RCLCPP_INFO(this->get_logger(), "Pub:%lf", pose_msg->pose.position.y);
//                     path_array_num_[i] += 1;
//                     path_attentive_last_pose = {pose_msg->pose.position.x, pose_msg->pose.position.y};
//                     // vec_pose_publisher_[i]->publish(*pose_msg);
//                 }
//                 vec_pose_publisher_[i]->publish(*pose_msg);
//                 // });
//             }