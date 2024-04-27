#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>
#include <my_msgs/msg/order_request.hpp>
#include <cmath>
// #include <geometry_msgs/msg/pose.hpp>

namespace tlab
{

const size_t ROBOT_NUM = 2;

class RouteController : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> vec_pose_publisher_;
    rclcpp::Publisher<my_msgs::msg::RouteRequest>::SharedPtr route_request_publisher_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> calc_path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr attentive_path_subscription;
    rclcpp::Subscription<my_msgs::msg::OrderRequest>::SharedPtr order_request_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr point_timer_;

    size_t count_;
    std::vector<int> path_array_num_ = std::vector<int>(ROBOT_NUM, 0);
    std::vector<int> order_array_num_ = std::vector<int>(ROBOT_NUM, 0);
    std::vector<bool> picking_condition = std::vector<bool>(ROBOT_NUM, true);
    // std::vector<float> path_attentive_last_pose{};
    // std::vector<std::vector<float>> path_attentive_last_pose(ROBOT_NUM, std::vector<float>(0, 0));
    std::vector<std::vector<float>> path_attentive_last_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));

    std::vector<nav_msgs::msg::Path> all_robot_path;
    std::vector<nav_msgs::msg::Path> sub_order_paths = std::vector<nav_msgs::msg::Path>(ROBOT_NUM);
    // nav_msgs::msg::Path sub_order_paths;

    std::vector<int> robot_mode = std::vector<int>(ROBOT_NUM, 0);
    std::vector<std::vector<float>> current_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> next_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> diff_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> current_move_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));
    std::vector<std::vector<float>> next_move_pose = std::vector<std::vector<float>>(ROBOT_NUM, std::vector<float>(2, 0));

    float inv_route_2 = 1 / sqrt(2.0);
    float v = 1; //[m/s]

    // std::vector<bool> picking_condition = {true, true, true};

public:
    RouteController(const rclcpp::NodeOptions& options) : RouteController("", options) {}
    RouteController(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_controller_node", name_space, options), all_robot_path(ROBOT_NUM)
    {
        using namespace std::chrono_literals;
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_topic_test", rclcpp::QoS(10));
        route_request_publisher_ = this->create_publisher<my_msgs::msg::RouteRequest>("route_request_topic_test", rclcpp::QoS(10));
        static auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.frame_id = "map"; // 基準座標を決めている、送るときだけ

        // オーダーのパスをサブスクライブ
        order_request_subscription = this->create_subscription<my_msgs::msg::OrderRequest>("order_request_topic_test", rclcpp::QoS(10), [&](const my_msgs::msg::OrderRequest::SharedPtr order_request_msg) {
            // std::cout << order_request_msg->order_paths.poses.size() << std::endl;
            for (int i = 0; i < order_request_msg->order_paths.poses.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = order_request_msg->order_paths.poses[i].pose.position.x;
                pose.pose.position.y = order_request_msg->order_paths.poses[i].pose.position.y;
                sub_order_paths[order_request_msg->robot_no].poses.push_back(pose);
            }
            // 最初のroute_request.msgを送信
            auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
            route_request_msg->robot_no = order_request_msg->robot_no;
            route_request_msg->start.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.x;
            route_request_msg->start.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position.y;
            route_request_msg->goal.pose.position.x = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no] + 1].pose.position.x;
            route_request_msg->goal.pose.position.y = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no] + 1].pose.position.y;

            for (int i = 0; i < ROBOT_NUM; i++) {
                if (route_request_msg->robot_no != i && !all_robot_path[i].poses.empty()) {
                    nav_msgs::msg::Path remaining_paths;
                    for (int j = path_array_num_[i]; j < all_robot_path[i].poses.size(); j++) {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.pose.position.x = all_robot_path[i].poses[j].pose.position.x;
                        pose.pose.position.y = all_robot_path[i].poses[j].pose.position.y;
                        remaining_paths.poses.push_back(pose);
                    }
                    route_request_msg->other_path.push_back(remaining_paths);
                }
            }

            order_array_num_[route_request_msg->robot_no] += 1;

            route_request_publisher_->publish(*route_request_msg);
        });

        // 該当するロボットのcalc_pathを受信
        for (size_t n = 0; n < ROBOT_NUM; n++) {
            calc_path_subscription_.push_back(this->create_subscription<nav_msgs::msg::Path>("calc_path_" + std::to_string(n), rclcpp::QoS(10), [&, n](const nav_msgs::msg::Path::SharedPtr msg) { all_robot_path[n] = *msg; }));
        }

        // 各pose の送受信のために名前決め
        for (size_t i = 0; i < ROBOT_NUM; i++) {
            vec_pose_publisher_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_" + std::to_string(i), rclcpp::QoS(10)));
        }

        point_timer_ = this->create_wall_timer(10ms, [&]() {
            for (int i = 0; i < ROBOT_NUM; i++) {
                auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
                pose_msg->header.frame_id = "map";
                if (robot_mode[i] == 0) {
                    if (all_robot_path[i].poses.empty()) {
                        continue;
                    }
                    else {
                        robot_mode[i] = 1;
                        current_pose[i] = {all_robot_path[i].poses[path_array_num_[i]].pose.position.x, all_robot_path[i].poses[path_array_num_[i]].pose.position.y};
                        next_pose[i] = {all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.x, all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.y};
                        current_move_pose[i] = current_pose[i];
                        path_array_num_[i] += 1;
                    }
                }
                if (robot_mode[i] == 2) {
                    if (all_robot_path[i].poses.empty()) {
                        pose_msg->pose.position.x = next_pose[i][0];
                        pose_msg->pose.position.y = next_pose[i][1];
                        vec_pose_publisher_[i]->publish(*pose_msg);
                        continue;
                    }
                    else {
                        robot_mode[i] = 1;
                        current_pose[i] = {all_robot_path[i].poses[path_array_num_[i]].pose.position.x, all_robot_path[i].poses[path_array_num_[i]].pose.position.y};
                        next_pose[i] = {all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.x, all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.y};
                        current_move_pose[i] = current_pose[i];
                        path_array_num_[i] += 1;
                    }
                }
                if (robot_mode[i] == 1) {
                    if (std::sqrt(std::pow(next_pose[i][0] - current_move_pose[i][0], 2) + std::pow(next_pose[i][1] - current_move_pose[i][1], 2)) < v * 0.1) {
                        if (next_pose[i][0] == all_robot_path[i].poses.back().pose.position.x && next_pose[i][1] == all_robot_path[i].poses.back().pose.position.y) {
                            robot_mode[i] = 2;
                            pose_msg->pose.position.x = next_pose[i][0];
                            pose_msg->pose.position.y = next_pose[i][1];
                            vec_pose_publisher_[i]->publish(*pose_msg);
                            path_array_num_[i] = 0;
                            all_robot_path[i].poses.clear();
                            if (next_pose[i][0] == sub_order_paths[i].poses.back().pose.position.x && next_pose[i][1] == sub_order_paths[i].poses.back().pose.position.y) {
                                continue;
                            }
                            auto route_request_msg = std::make_shared<my_msgs::msg::RouteRequest>();
                            route_request_msg->robot_no = i;
                            route_request_msg->start.pose.position = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no]].pose.position;
                            route_request_msg->goal.pose.position = sub_order_paths[route_request_msg->robot_no].poses[order_array_num_[route_request_msg->robot_no] + 1].pose.position;
                            for (int i = 0; i < ROBOT_NUM; i++) {
                                if (route_request_msg->robot_no != i && !all_robot_path[i].poses.empty()) {
                                    nav_msgs::msg::Path remaining_paths;
                                    for (int j = path_array_num_[i]; j < all_robot_path[i].poses.size(); j++) {
                                        geometry_msgs::msg::PoseStamped pose;
                                        pose.pose.position = all_robot_path[i].poses[j].pose.position;
                                        remaining_paths.poses.push_back(pose);
                                    }
                                    route_request_msg->other_path.push_back(remaining_paths);
                                }
                            }
                            order_array_num_[route_request_msg->robot_no] += 1;
                            route_request_publisher_->publish(*route_request_msg);
                            // pose_msg->pose.position.x = next_pose[i][0];
                            // pose_msg->pose.position.y = next_pose[i][1];
                            // vec_pose_publisher_[i]->publish(*pose_msg);
                            // path_array_num_[i] = 0;
                            // all_robot_path[i].poses.clear();
                            continue;
                        }
                        current_pose[i] = next_pose[i];
                        next_pose[i] = {all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.x, all_robot_path[i].poses[path_array_num_[i] + 1].pose.position.y};
                        path_array_num_[i] += 1;
                    }
                    diff_pose[i] = {next_pose[i][0] - current_pose[i][0], next_pose[i][1] - current_pose[i][1]};
                    if (diff_pose[i][0] + diff_pose[i][1] != 0) {
                        next_move_pose[i] = {current_move_pose[i][0] + inv_route_2 * v * 0.1 * diff_pose[i][0], current_move_pose[i][1] + inv_route_2 * v * 0.1 * diff_pose[i][1]};
                    }
                    else {
                        next_move_pose[i] = {current_move_pose[i][0] + v * 0.1 * diff_pose[i][0], current_move_pose[i][1] + v * 0.1 * diff_pose[i][1]};
                    }
                    current_move_pose[i] = next_move_pose[i];
                    pose_msg->pose.position.x = next_move_pose[i][0];
                    pose_msg->pose.position.y = next_move_pose[i][1];
                    // std::cout << i << "  " << next_move_pose[i][0] << " " << next_move_pose[i][1] << std::endl;
                    vec_pose_publisher_[i]->publish(*pose_msg);
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
//                 else {
//                     if (all_robot_path[i].poses.empty() && (!(path_attentive_last_pose[i][0] == 0 && path_attentive_last_pose[i][1] == 0))) {
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