#pragma once
#pragma once #pragma onceone_more_chanceone_more_chance

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <my_msgs/msg/pick_request.hpp>
#include <my_msgs/msg/re_route_request.hpp>

#include "a_star.hpp"
#include "g_layor.hpp"

#include <cmath>
#include <chrono>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <my_msgs/msg/pick_request.hpp>
#include <my_msgs/msg/re_route_request.hpp>

#include "a_star.hpp"
#include "g_layor.hpp"

#include <cmath>
#include <chrono>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <my_msgs/msg/route_request.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <my_msgs/msg/pick_request.hpp>
#include <my_msgs/msg/re_route_request.hpp>

#include "a_star.hpp"
#include "g_layor.hpp"

#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>

namespace tlab
{

struct cross_main_infother {
    std::vector<int> vec_robot_no;
    std::vector<int> vec_robot_no_index;
    std::vector<float> vec_cross_start_dis;
    std::vector<float> vec_cross_end_dis;
    std::vector<int> vec_cross_start_index;
    std::vector<int> vec_cross_end_index;
    std::vector<int> vec_start_ROW;
    std::vector<int> vec_start_COL;
    std::vector<int> vec_end_ROW;
    std::vector<int> vec_end_COL;
    std::vector<int> vec_path_counter;
    std::vector<int> vec_path_index;
};

struct cross_infmain_other {
    std::vector<float> vec_robot_no;
    std::vector<int> vec_cross_start_index;
    std::vector<int> vec_cross_end_index;
};

struct attentive_cross {
    std::vector<int> vec_start_ROW;
    std::vector<int> vec_start_COL;
    std::vector<int> vec_end_ROW;
    std::vector<int> vec_end_COL;
};

const size_t ROBOT_NUM = 20;

class RouteCalculator : public rclcpp::Node {
private:
    rclcpp::Subscription<my_msgs::msg::RouteRequest>::SharedPtr route_request_subscription;
    rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr infmap_subscription;
    rclcpp::Subscription<my_msgs::msg::PickRequest>::SharedPtr pick_point_subscription;
    rclcpp::Publisher<my_msgs::msg::ReRouteRequest>::SharedPtr re_route_request_publisher_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> calc_path_publisher_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> calc_MATpath_publisher_;

    std::vector<std::vector<int>> map_grid_matrix;
    std::vector<std::vector<int>> all_infmap;
    std::vector<std::vector<int>> Astar_map;
    std::vector<std::vector<std::vector<float>>> g_foundation_list_defo;
    std::vector<std::vector<std::vector<float>>> g_foundation_list;
    int re_route_calc_count = 0;

    std::vector<float> calc_time_vec = std::vector<float>(ROBOT_NUM, 0);

    std::vector<std::vector<float>> calc_time_csv_list = std::vector<std::vector<float>>(ROBOT_NUM);
    std::string csv_path = "/home/haru/ros2_ws/calc_time_output";

    int64_t resolution;
    int64_t height_m;
    int64_t width_m;
    int64_t height_row;
    int64_t width_col;

    float sqrt_2 = std::sqrt(2.0);
    float inflate_m;
    int path_number;
    int count_tyohuku = 0;
    int dakyo_count = 0;

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

    // startやgoalをinfはmode0,up_pickをinfはmode1,down_pickをinfはmode2
    void create_poses_inf(int height_row, int width_col, std::shared_ptr<my_msgs::msg::PickRequest_<std::allocator<void>>> pick_point_msg, int inf6, std::vector<std::vector<int>>& all_infmap)
    {
        std::vector<std::vector<float>> attentive_map = std::vector<std::vector<float>>(height_row + 2, std::vector<float>(width_col + 2, 0));
        // startとgoal
        for (int i = 0; i < pick_point_msg->start_goal.poses.size(); i++) {
            std::vector<float> pos = {pick_point_msg->start_goal.poses[i].pose.position.x, pick_point_msg->start_goal.poses[i].pose.position.y};
            std::vector<int> MATpos = to_MATpos_m(pos, height_row, resolution);

            int min_row = MATpos[0] - inf6;
            int max_row = MATpos[0] + inf6;
            int min_col = MATpos[1] - inf6;
            int max_col = MATpos[1] + inf6;

            if (min_row < 0) {
                min_row = 0;
            }
            if (min_col < 0) {
                min_col = 0;
            }
            if (max_row > height_row) {
                max_row = height_row;
            }
            if (max_col > width_col) {
                max_col = width_col;
            }

            attentive_map[min_row][min_col] += 1;
            attentive_map[max_row + 1][max_col + 1] += 1;
            attentive_map[max_row + 1][min_col] += -1;
            attentive_map[min_row][max_col + 1] += -1;
        }
        for (int i = 0; i < pick_point_msg->up_pick.poses.size(); i++) {
            std::vector<float> pos = {pick_point_msg->up_pick.poses[i].pose.position.x, pick_point_msg->up_pick.poses[i].pose.position.y};
            std::vector<int> MATpos = to_MATpos_m(pos, height_row, resolution);

            int min_row = MATpos[0] - inf6;
            int max_row = MATpos[0];
            int min_col = MATpos[1] - 5;
            int max_col = MATpos[1] + 5;

            if (min_row < 0) {
                min_row = 0;
            }
            if (min_col < 0) {
                min_col = 0;
            }
            if (max_row > height_row) {
                max_row = height_row;
            }
            if (max_col > width_col) {
                max_col = width_col;
            }

            attentive_map[min_row][min_col] += 1;
            attentive_map[max_row + 1][max_col + 1] += 1;
            attentive_map[max_row + 1][min_col] += -1;
            attentive_map[min_row][max_col + 1] += -1;
        }

        for (int i = 0; i < pick_point_msg->down_pick.poses.size(); i++) {
            std::vector<float> pos = {pick_point_msg->down_pick.poses[i].pose.position.x, pick_point_msg->down_pick.poses[i].pose.position.y};
            std::vector<int> MATpos = to_MATpos_m(pos, height_row, resolution);

            int min_row = MATpos[0];
            int max_row = MATpos[0] + inf6;
            int min_col = MATpos[1] - 5;
            int max_col = MATpos[1] + 5;

            if (min_row < 0) {
                min_row = 0;
            }
            if (min_col < 0) {
                min_col = 0;
            }
            if (max_row > height_row) {
                max_row = height_row;
            }
            if (max_col > width_col) {
                max_col = width_col;
            }

            attentive_map[min_row][min_col] += 1;
            attentive_map[max_row + 1][max_col + 1] += 1;
            attentive_map[max_row + 1][min_col] += -1;
            attentive_map[min_row][max_col + 1] += -1;
        }

        // int count = 0;
        std::vector<std::vector<float>> calc_map = std::vector<std::vector<float>>(height_row + 3, std::vector<float>(width_col + 3, 0));
        for (int i = 0; i < height_row + 1; i++) {
            for (int j = 0; j < width_col + 1; j++) {
                calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];
                if (calc_map[i + 1][j + 1] != 0) {
                    all_infmap[i][j] = 1;
                    // count++;
                }
            }
        }
    }

    void delete_pose_inf(int pari, std::vector<float> pos_, std::vector<std::vector<int>>& Astar_map, int height_row, int width_col, int resolution, int inf6)
    {
        std::vector<std::vector<float>> attentive_map = std::vector<std::vector<float>>(height_row + 2, std::vector<float>(width_col + 2, 0));
        std::vector<int> MATpos = to_MATpos_m(pos_, height_row, resolution);
        int min_row;
        int max_row;
        int min_col;
        int max_col;
        if (pari == 0) {
            min_row = MATpos[0] - inf6;
            max_row = MATpos[0] + inf6;
            min_col = MATpos[1] - inf6;
            max_col = MATpos[1] + inf6;
        }
        else if (pari == 1) {
            min_row = MATpos[0];
            max_row = MATpos[0] + inf6;
            min_col = MATpos[1] - 5;
            max_col = MATpos[1] + 5;
        }
        else if (pari == 2) {
            min_row = MATpos[0] - inf6;
            max_row = MATpos[0];
            min_col = MATpos[1] - 5;
            max_col = MATpos[1] + 5;
        }
        if (min_row < 0) {
            min_row = 0;
        }
        if (min_col < 0) {
            min_col = 0;
        }
        if (max_row > height_row) {
            max_row = height_row;
        }
        if (max_col > width_col) {
            max_col = width_col;
        }

        attentive_map[min_row][min_col] += 1;
        attentive_map[max_row + 1][max_col + 1] += 1;
        attentive_map[max_row + 1][min_col] += -1;
        attentive_map[min_row][max_col + 1] += -1;

        std::vector<std::vector<float>> calc_map = std::vector<std::vector<float>>(height_row + 3, std::vector<float>(width_col + 3, 0));
        for (int i = 0; i < height_row + 1; i++) {
            for (int j = 0; j < width_col + 1; j++) {
                calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];
                if (calc_map[i + 1][j + 1] != 0) {
                    Astar_map[i][j] = 0;
                }
            }
        }
    }

    void create_other_infgrid(const nav_msgs::msg::Path& other_path, int inf6, std::vector<std::vector<int>>& other_infgrid, int height_row, int width_col, int skip_index)
    {
        // attentive_mapを初期化する
        std::vector<std::vector<float>> attentive_map(height_row + 2, std::vector<float>(width_col + 2, 0));

        // other_pathの各ポーズについて処理する
        // for (const auto& pose : other_path.poses) {
        for (int i = skip_index; i < other_path.poses.size(); i++) {
            int x = round(other_path.poses[i].pose.position.x);
            int y = round(other_path.poses[i].pose.position.y);
            // int x = round(pose.pose.position.x);
            // int y = round(pose.pose.position.y);

            int min_row = x - inf6;
            int max_row = x + inf6;
            int min_col = y - inf6;
            int max_col = y + inf6;

            // 境界を確認して修正する
            min_row = std::max(min_row, 0);
            min_col = std::max(min_col, 0);
            max_row = std::min(max_row, height_row);
            max_col = std::min(max_col, width_col);

            // attentive_mapを更新する
            attentive_map[min_row][min_col] += 1;
            attentive_map[max_row + 1][max_col + 1] += 1;
            attentive_map[max_row + 1][min_col] -= 1;
            attentive_map[min_row][max_col + 1] -= 1;
        }

        // calc_mapを初期化する
        std::vector<std::vector<float>> calc_map(height_row + 2, std::vector<float>(width_col + 2, 0));

        // calc_mapを計算する
        for (int i = 0; i < height_row + 1; i++) {
            for (int j = 0; j < width_col + 1; j++) {
                calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];

                // calc_mapの値が0でない場合、other_infgridを更新する
                if (calc_map[i + 1][j + 1] != 0 && i < height_row && j < width_col) {
                    other_infgrid[i][j] = 1;
                }
            }
        }
    }

    void create_main_infgrid(const std::vector<std::vector<int>>& MATpath, int inf6, std::vector<std::vector<int>>& main_infgrid, int height_row, int width_col)
    {
        // attentive_mapを初期化する
        std::vector<std::vector<float>> attentive_map(height_row + 2, std::vector<float>(width_col + 2, 0));

        // other_pathの各ポーズについて処理する
        // for (const auto& pose : other_path.poses) {
        for (int i = 0; i < MATpath.size(); i++) {

            int min_row = MATpath[i][0] - inf6;
            int max_row = MATpath[i][0] + inf6;
            int min_col = MATpath[i][1] - inf6;
            int max_col = MATpath[i][1] + inf6;

            // 境界を確認して修正する
            min_row = std::max(min_row, 0);
            min_col = std::max(min_col, 0);
            max_row = std::min(max_row, height_row);
            max_col = std::min(max_col, width_col);

            // attentive_mapを更新する
            attentive_map[min_row][min_col] += 1;
            attentive_map[max_row + 1][max_col + 1] += 1;
            attentive_map[max_row + 1][min_col] -= 1;
            attentive_map[min_row][max_col + 1] -= 1;
        }

        // calc_mapを初期化する
        std::vector<std::vector<float>> calc_map(height_row + 2, std::vector<float>(width_col + 2, 0));

        // calc_mapを計算する
        for (int i = 0; i < height_row + 1; i++) {
            for (int j = 0; j < width_col + 1; j++) {
                calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];

                // calc_mapの値が0でない場合、other_infgridを更新する
                if (calc_map[i + 1][j + 1] != 0 && i < height_row && j < width_col) {
                    main_infgrid[i][j] = 1;
                }
            }
        }
    }

    void cross_judge(const std::vector<std::vector<int>>& other_infgrid, const std::vector<std::vector<int>>& MATpath, int other_robot_no, cross_main_infother& cross, int robot_no_index)
    {
        bool cross_mode = false;
        int chuning = 0;
        float dis_chuning = 0.12 * chuning;
        float dis = 0;
        float atten_dis;
        int atten_index;
        int path_count = 0;

        // bool key = false;
        // bool extend_key = false;

        for (int i = 1; i < MATpath.size(); i++) {
            if (other_infgrid[MATpath[i][0]][MATpath[i][1]] != 0 && cross_mode == false) {

                // std::cout << "cross:" << other_robot_no;
                // std::cout << "はじめ"
                //           << "::";
                // std::cout << "(" << MATpath[i - 1][0] << "," << MATpath[i - 1][1] << ")";
                atten_dis = (dis / 10) + dis_chuning;
                atten_index = i - 1;
                cross_mode = true;
                // if (key == true) {
                //     std::cout << "tasikameru" << std::endl;
                //     int dx = MATpath[i - 1][0] - cross.vec_end_ROW.back();
                //     int dy = MATpath[i - 1][1] - cross.vec_end_COL.back();
                //     float begin_end_dis = (std::sqrt(dx * dx + dy * dy)) / 10;
                //     std::cout << "begin_end_dis:" << begin_end_dis << std::endl;
                //     if (begin_end_dis < 1) {
                //         extend_key = true;
                //         std::cout << "結合承りました" << std::endl;
                //     }
                // }
            }

            if (other_infgrid[MATpath[i][0]][MATpath[i][1]] == 0 && cross_mode == true) {

                // if (extend_key == false) {
                cross.vec_robot_no_index.push_back(robot_no_index);
                cross.vec_robot_no.push_back(other_robot_no);

                cross.vec_cross_start_dis.push_back(atten_dis);
                cross.vec_cross_start_index.push_back(atten_index);
                cross.vec_start_ROW.push_back(MATpath[atten_index][0]);
                cross.vec_start_COL.push_back(MATpath[atten_index][1]);
                // std::cout << "おわり"
                //           << "::";
                // std::cout << "(" << MATpath[i][0] << "," << MATpath[i][1] << ")" << std::endl;
                cross.vec_cross_end_dis.push_back((dis / 10) - dis_chuning);
                cross.vec_cross_end_index.push_back(i);
                cross.vec_end_ROW.push_back(MATpath[i][0]);
                cross.vec_end_COL.push_back(MATpath[i][1]);
                cross_mode = false;
                path_count++;
                // key = true;
                // }
                // else {
                //     std::cout << "新しい終わり"
                //               << "::";
                //     std::cout << "(" << MATpath[i][0] << "," << MATpath[i][1] << ")" << std::endl;
                //     cross.vec_cross_end_dis.back() = (dis / 10) - dis_chuning;
                //     cross.vec_cross_end_index.back() = i;
                //     cross.vec_end_ROW.back() = MATpath[i][0];
                //     cross.vec_end_COL.back() = MATpath[i][1];
                //     cross_mode = false;
                //     extend_key = false;
                // }
            }

            dis = dis + std::hypot(MATpath[i][0] - MATpath[i - 1][0], MATpath[i][1] - MATpath[i - 1][1]);
        }

        if (cross_mode == true) {
            cross.vec_robot_no_index.push_back(robot_no_index);
            cross.vec_robot_no.push_back(other_robot_no);

            cross.vec_cross_start_dis.push_back(atten_dis);
            cross.vec_cross_start_index.push_back(atten_index);
            cross.vec_start_ROW.push_back(MATpath[atten_index][0]);
            cross.vec_start_COL.push_back(MATpath[atten_index][1]);

            // std::cout << "おわり(tokusyu)"
            //           << "::";
            // std::cout << "(" << MATpath[MATpath.size() - 1][0] << "," << MATpath[MATpath.size() - 1][1] << ")" << std::endl;
            cross.vec_cross_end_dis.push_back((dis / 10) - dis_chuning);
            cross.vec_cross_end_index.push_back(MATpath.size() - 1);
            cross.vec_end_ROW.push_back(MATpath[MATpath.size() - 1][0]);
            cross.vec_end_COL.push_back(MATpath[MATpath.size() - 1][1]);
            path_count++;
        }

        for (int i = 0; i < path_count; i++) {
            cross.vec_path_counter.push_back(path_count);
            cross.vec_path_index.push_back(robot_no_index);
        }
    }

    // void cross_adjust(const std::vector<std::vector<int>>& main_infgrid, const std::vector<nav_msgs::msg::Path>& other_paths, const std::vector<int>& ten_skip_index, const cross_main_infother& cross)
    // {
    //     for (int i = 0; i < cross.vec_path_index.size(); i++) {
    //         bool cross_mode = false;
    //         int chuning = 0;
    //         float dis_chuning = 0.12 * chuning;
    //         float dis = 0;
    //         float atten_dis;
    //         int atten_index, atten_pre_ROW, atten_pre_COL;
    //         int pre_ROW = -1, pre_COL = -1;
    //         int ROW, COL;
    //         int path_count = 0;
    //         attentive_cross cross_3;

    //         for (int j = ten_skip_index[cross.vec_path_index[i]]; j < other_paths[cross.vec_path_index[i]].poses.size(); j++) {
    //             ROW = round(other_paths[cross.vec_path_index[i]].poses[j].pose.position.x);
    //             COL = round(other_paths[cross.vec_path_index[i]].poses[j].pose.position.y);
    //             if (main_infgrid[ROW][COL] != 0 && cross_mode == false) {
    //                 if (pre_ROW == -1) {
    //                     pre_ROW = ROW;
    //                     pre_COL = COL;
    //                 }
    //                 atten_pre_ROW = pre_ROW;
    //                 atten_pre_COL = pre_COL;
    //                 cross_mode = true;
    //             }

    //             if (main_infgrid[ROW][COL] == 0 && cross_mode == true) {
    //                 cross_3.vec_start_ROW.push_back(atten_pre_ROW);
    //                 cross_3.vec_start_COL.push_back(atten_pre_COL);
    //                 cross_3.vec_end_ROW.push_back(ROW);
    //                 cross_3.vec_end_COL.push_back(COL);
    //                 cross_mode = false;
    //                 path_count++;

    //             }

    //             pre_ROW = ROW;
    //             pre_COL = COL;
    //         }

    //         if (cross_mode == true) {
    //             cross_3.vec_start_ROW.push_back(atten_pre_ROW);
    //             cross_3.vec_start_COL.push_back(atten_pre_COL);
    //             cross_3.vec_end_ROW.push_back(ROW);
    //             cross_3.vec_end_COL.push_back(COL);
    //             path_count++;
    //         }

    //         if(path_count==cross.vec_path_counter[i]){
    //             //ok
    //         }else if(path_count<cross.vec_path_counter[i]){
    //             //ketugou(cross gawa)
    //         }else{
    //             //ketugou(corss_2 gawa)
    //         }

    //     }
    // }

    void cross2_judge(const std::vector<std::vector<int>>& main_infgrid, const std::vector<nav_msgs::msg::Path>& other_paths, const std::vector<int>& ten_skip_index, cross_infmain_other& cross_2, const cross_main_infother& cross, int& count, bool& main_key)
    {
        for (int i = 0; i < cross.vec_robot_no_index.size(); i++) {
            bool cross_mode = false;
            int chuning = 0;
            float dis_chuning = 0.12 * chuning;
            float cross_mergin = 0.3;
            float dis = 0;
            float atten_dis;
            int atten_index, atten_pre_ROW, atten_pre_COL;

            int pre_ROW = -1, pre_COL = -1;
            int ROW, COL;

            int accordance_count = 0;

            for (int j = ten_skip_index[cross.vec_robot_no_index[i]]; j < other_paths[cross.vec_robot_no_index[i]].poses.size(); j++) {
                ROW = round(other_paths[cross.vec_robot_no_index[i]].poses[j].pose.position.x);
                COL = round(other_paths[cross.vec_robot_no_index[i]].poses[j].pose.position.y);
                if (main_infgrid[ROW][COL] != 0 && cross_mode == false) {
                    if (pre_ROW == -1) {
                        pre_ROW = ROW;
                        pre_COL = COL;
                    }
                    atten_dis = (dis / 10) + dis_chuning;
                    atten_index = j - 1;
                    atten_pre_ROW = pre_ROW;
                    atten_pre_COL = pre_COL;
                    // std::cout << "cross_2:" << cross.vec_robot_no[i];
                    // std::cout << "はじめ"
                    //           << "::";
                    // std::cout << "(" << atten_pre_ROW << "," << atten_pre_COL << ")";
                    cross_mode = true;
                }

                if (main_infgrid[ROW][COL] == 0 && cross_mode == true) {

                    // std::cout << "おわり"
                    //           << "::";
                    // std::cout << "(" << ROW << "," << COL << ")" << std::endl;
                    float end_dis = (dis / 10) - dis_chuning;

                    accordance_count++;

                    // ここでcross内のどれとペアになるかを、交差、順方向、逆方向重複なのか見極める、そして、crossにおけるother_robot_noも記憶
                    // もし交差or順方向重複なら
                    int dx = atten_pre_ROW - cross.vec_start_ROW[i];
                    int dy = atten_pre_COL - cross.vec_start_COL[i];
                    float distance_ss = (std::sqrt(dx * dx + dy * dy)) / 10;

                    dx = atten_pre_ROW - cross.vec_end_ROW[i];
                    dy = atten_pre_COL - cross.vec_end_COL[i];
                    float distance_se = (std::sqrt(dx * dx + dy * dy)) / 10;

                    dx = ROW - cross.vec_end_ROW[i];
                    dy = COL - cross.vec_end_COL[i];
                    float distance_ee = (std::sqrt(dx * dx + dy * dy)) / 10;

                    dx = ROW - cross.vec_start_ROW[i];
                    dy = COL - cross.vec_start_COL[i];
                    float distance_es = (std::sqrt(dx * dx + dy * dy)) / 10;

                    // std::cout << "E1:" << end_dis + cross_mergin << " S3:" << cross.vec_cross_start_dis[i] << " S1:" << atten_dis << " E3:" << cross.vec_cross_end_dis[i] + cross_mergin << std::endl;
                    if (distance_ss < 2 && distance_ee < 2) {                                                                                 // 交差or順方向重複
                        if (end_dis + cross_mergin < cross.vec_cross_start_dis[i] || atten_dis > cross.vec_cross_end_dis[i] + cross_mergin) { // 衝突しない条件（一回目のふるい）
                            // std::cout << cross.vec_robot_no[i] << ":衝突しない" << std::endl;
                        }
                        else if (cross.vec_cross_end_dis[i] - cross.vec_cross_start_dis[i] < 3) { // 交差の場合
                            // std::cout << cross.vec_robot_no[i] << ":交差衝突" << std::endl;
                            cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                            cross_2.vec_cross_start_index.push_back(atten_index);
                            cross_2.vec_cross_end_index.push_back(j);
                        }
                        else { //  順方向重複で、厳密に衝突を見極める
                            count++;
                            if (std::abs(cross.vec_cross_start_dis[i] - atten_dis) < 1.5) { // S3とS1の距離、ただし、chuningによって進んでいることに注意
                                // std::cout << cross.vec_robot_no[i] << ":順方向衝突" << std::endl;
                                cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                                cross_2.vec_cross_start_index.push_back(atten_index);
                                cross_2.vec_cross_end_index.push_back(atten_index + 20); // 重複しはじめ約3mを重み増
                            }
                            else {
                                // std::cout << cross.vec_robot_no[i] << ":順方向重複だけど衝突しない" << std::endl;
                            }
                        }
                    }
                    else if (distance_se < 2 && distance_es < 2) {                                                                            // 交差or逆方向重複
                        if (end_dis + cross_mergin < cross.vec_cross_start_dis[i] || atten_dis > cross.vec_cross_end_dis[i] + cross_mergin) { // 衝突しない条件（一回目のふるい）
                            // std::cout << cross.vec_robot_no[i] << ":衝突しない" << std::endl;
                        }
                        else if (cross.vec_cross_end_dis[i] - cross.vec_cross_start_dis[i] < 3) { // 交差の場合
                            // std::cout << cross.vec_robot_no[i] << ":交差衝突" << std::endl;
                            cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                            cross_2.vec_cross_start_index.push_back(atten_index);
                            cross_2.vec_cross_end_index.push_back(j);
                        }
                        else { //  順方向重複で、厳密に衝突を見極める
                            // std::cout << cross.vec_robot_no[i] << ":逆方向衝突" << std::endl;
                            count++;
                            cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                            cross_2.vec_cross_start_index.push_back(atten_index);
                            cross_2.vec_cross_end_index.push_back(j);
                        }
                    }
                    else if (end_dis + cross_mergin < cross.vec_cross_start_dis[i] || atten_dis > cross.vec_cross_end_dis[i] + cross_mergin) {
                        // std::cout << cross.vec_robot_no[i] << ":衝突しない(すり抜け)" << std::endl;
                    }
                    else { // どちらでもない,1経路で2以上の交差
                        // std::cout << cross.vec_robot_no[i] << ":どちらでもない" << std::endl;
                    }

                    cross_mode = false;
                }

                if (pre_ROW != -1) {
                    dis = dis + std::hypot(ROW - pre_ROW, COL - pre_COL);
                }
                pre_ROW = ROW;
                pre_COL = COL;
            }

            if (cross_mode == true) {
                // E1の情報を取得
                float end_dis = (dis / 10) - dis_chuning;
                // std::cout << "おわり(tokusyu)"
                //           << "::";
                // std::cout << "(" << ROW << "," << COL << ")" << std::endl;

                accordance_count++;
                // ここでcross内のどれとペアになるかを、交差、順方向、逆方向重複なのか見極める、そして、crossにおけるother_robot_noも記憶
                // もし交差or順方向重複なら
                int dx = atten_pre_ROW - cross.vec_start_ROW[i];
                int dy = atten_pre_COL - cross.vec_start_COL[i];
                float distance_ss = (std::sqrt(dx * dx + dy * dy)) / 10;

                dx = atten_pre_ROW - cross.vec_end_ROW[i];
                dy = atten_pre_COL - cross.vec_end_COL[i];
                float distance_se = (std::sqrt(dx * dx + dy * dy)) / 10;

                dx = ROW - cross.vec_end_ROW[i];
                dy = COL - cross.vec_end_COL[i];
                float distance_ee = (std::sqrt(dx * dx + dy * dy)) / 10;

                dx = ROW - cross.vec_start_ROW[i];
                dy = COL - cross.vec_start_COL[i];
                float distance_es = (std::sqrt(dx * dx + dy * dy)) / 10;

                // std::cout << "E1:" << end_dis + cross_mergin << " S3:" << cross.vec_cross_start_dis[i] << " S1:" << atten_dis << " E3:" << cross.vec_cross_end_dis[i] + cross_mergin << std::endl;
                if (distance_ss < 2 && distance_ee < 2) {                                                                                 // 交差or順方向重複
                    if (end_dis + cross_mergin < cross.vec_cross_start_dis[i] || atten_dis > cross.vec_cross_end_dis[i] + cross_mergin) { // 衝突しない条件（一回目のふるい）
                        // std::cout << cross.vec_robot_no[i] << ":衝突しない" << std::endl;
                    }
                    else if (cross.vec_cross_end_dis[i] - cross.vec_cross_start_dis[i] < 3) { // 交差の場合
                        // std::cout << cross.vec_robot_no[i] << ":交差衝突" << std::endl;
                        cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                        cross_2.vec_cross_start_index.push_back(atten_index);
                        cross_2.vec_cross_end_index.push_back(other_paths[cross.vec_robot_no_index[i]].poses.size() - 1);
                    }
                    else { //  順方向重複で、厳密に衝突を見極める
                        // std::cout << "順方向重複" << std::endl;
                        count++;
                        if (std::abs(cross.vec_cross_start_dis[i] - atten_dis) < 1.5) { // S3とS1の距離、ただし、chuningによって進んでいることに注意
                            // std::cout << cross.vec_robot_no[i] << ":順方向衝突" << std::endl;
                            cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                            cross_2.vec_cross_start_index.push_back(atten_index);
                            cross_2.vec_cross_end_index.push_back(atten_index + 20); // 重複しはじめ約3mを重み増
                        }
                        else {
                            // std::cout << cross.vec_robot_no[i] << ":順方向重複だけど衝突しない" << std::endl;
                        }
                    }
                }
                else if (distance_se < 2 && distance_es < 2) {                                                                            // 交差or逆方向重複
                    if (end_dis + cross_mergin < cross.vec_cross_start_dis[i] || atten_dis > cross.vec_cross_end_dis[i] + cross_mergin) { // 衝突しない条件（一回目のふるい）
                        // std::cout << cross.vec_robot_no[i] << ":衝突しない" << std::endl;
                    }
                    else if (cross.vec_cross_end_dis[i] - cross.vec_cross_start_dis[i] < 3) { // 交差の場合
                        // std::cout << cross.vec_robot_no[i] << ":交差衝突" << std::endl;
                        cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                        cross_2.vec_cross_start_index.push_back(atten_index);
                        cross_2.vec_cross_end_index.push_back(other_paths[cross.vec_robot_no_index[i]].poses.size() - 1);
                    }
                    else { //  順方向重複で、厳密に衝突を見極める
                        // std::cout << cross.vec_robot_no[i] << ":逆方向衝突" << std::endl;
                        count++;
                        cross_2.vec_robot_no.push_back(cross.vec_robot_no_index[i]);
                        cross_2.vec_cross_start_index.push_back(atten_index);
                        cross_2.vec_cross_end_index.push_back(other_paths[cross.vec_robot_no_index[i]].poses.size() - 1);
                    }
                }
                else if (end_dis + cross_mergin < cross.vec_cross_start_dis[i] || atten_dis > cross.vec_cross_end_dis[i] + cross_mergin) {
                    // std::cout << cross.vec_robot_no[i] << ":衝突しない(すり抜け)" << std::endl;
                }
                else { // どちらでもない,1経路で2以上の交差
                    // std::cout << cross.vec_robot_no[i] << ":どちらでもない" << std::endl;
                }
            }

            if (accordance_count != cross.vec_path_counter[i]) {
                // std::cout << "huittiniyoru_syototu" << std::endl;
                main_key = true;
            }
        }
    }

    void crossPlusCost(std::vector<std::vector<std::vector<float>>>& g_foundation_list, int inf6, const cross_infmain_other& cross_2, const std::vector<nav_msgs::msg::Path>& other_paths, int height_row, int width_col)
    {
        std::vector<std::vector<float>> attentive_map = std::vector<std::vector<float>>(height_row + 2, std::vector<float>(width_col + 2, 0));
        for (int i = 0; i < cross_2.vec_robot_no.size(); i++) {
            for (int j = cross_2.vec_cross_start_index[i]; j < cross_2.vec_cross_end_index[i]; j++) {
                std::vector<float> MATpos = {other_paths[cross_2.vec_robot_no[i]].poses[j].pose.position.x, other_paths[cross_2.vec_robot_no[i]].poses[j].pose.position.y};
                int min_row = MATpos[0] - inf6;
                int max_row = MATpos[0] + inf6;
                int min_col = MATpos[1] - inf6;
                int max_col = MATpos[1] + inf6;
                if (min_row < 0) {
                    min_row = 0;
                }
                if (min_col < 0) {
                    min_col = 0;
                }
                if (max_row > height_row) {
                    max_row = height_row;
                }
                if (max_col > width_col) {
                    max_col = width_col;
                }
                attentive_map[min_row][min_col] += 1;
                attentive_map[max_row + 1][max_col + 1] += 1;
                attentive_map[max_row + 1][min_col] += -1;
                attentive_map[min_row][max_col + 1] += -1;
            }
        }
        std::vector<std::vector<float>> calc_map = std::vector<std::vector<float>>(height_row + 3, std::vector<float>(width_col + 3, 0));
        for (int i = 0; i < height_row + 1; i++) {
            for (int j = 0; j < width_col + 1; j++) {
                calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];
                if (calc_map[i + 1][j + 1] != 0) {
                    for (int k = 0; k < 8; k++) {
                        g_foundation_list[k][i][j] += 10;
                    }
                }
            }
        }
    }

    void firstMainPlusCost(std::vector<std::vector<std::vector<float>>>& g_foundation_list, int inf6, const std::vector<std::vector<int>>& MATpath, int height_row, int width_col)
    {
        std::vector<std::vector<float>> attentive_map = std::vector<std::vector<float>>(height_row + 2, std::vector<float>(width_col + 2, 0));

        for (int j = 0; j < MATpath.size(); j++) {
            int min_row = MATpath[j][0] - inf6;
            int max_row = MATpath[j][0] + inf6;
            int min_col = MATpath[j][1] - inf6;
            int max_col = MATpath[j][1] + inf6;
            if (min_row < 0) {
                min_row = 0;
            }
            if (min_col < 0) {
                min_col = 0;
            }
            if (max_row > height_row) {
                max_row = height_row;
            }
            if (max_col > width_col) {
                max_col = width_col;
            }
            attentive_map[min_row][min_col] += 1;
            attentive_map[max_row + 1][max_col + 1] += 1;
            attentive_map[max_row + 1][min_col] += -1;
            attentive_map[min_row][max_col + 1] += -1;
        }

        std::vector<std::vector<float>> calc_map = std::vector<std::vector<float>>(height_row + 3, std::vector<float>(width_col + 3, 0));
        for (int i = 0; i < height_row + 1; i++) {
            for (int j = 0; j < width_col + 1; j++) {
                calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];
                if (calc_map[i + 1][j + 1] != 0) {
                    for (int k = 0; k < 8; k++) {
                        g_foundation_list[k][i][j] += 2;
                    }
                }
            }
        }
    }

    void writeCSV(const std::string& path, const std::vector<std::vector<float>>& output)
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
        oss << "calc_time_AMR" << ROBOT_NUM << "_" << std::put_time(&now_tm, "%Y年%m月%d日%H時%M分%S秒") << ".csv";
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

public:
    RouteCalculator(const rclcpp::NodeOptions& options) : RouteCalculator("", options) {}
    RouteCalculator(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_calcurator_node", name_space, options)
    {
        using namespace std::chrono_literals;

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
        static int inf6 = robot_size * resolution;
        re_route_request_publisher_ = this->create_publisher<my_msgs::msg::ReRouteRequest>("re_route_request_topic_test", rclcpp::QoS(10));

        g_foundation_list_defo = g_layor_.GfoundList_createIni(height_row, width_col, sqrt_2);

        float revision = (0.5 / resolution);

        // MAP情報をサブスクリプション
        infmap_subscription = this->create_subscription<nav_msgs::msg::GridCells>("infmap_topic_test", rclcpp::QoS(10), [&](const nav_msgs::msg::GridCells::SharedPtr infgrid_msg) {
            if (map_grid_matrix.size() != 0) {
                return;
            }

            std::vector<float> pos;
            std::vector<int> MATpos;
            map_grid_matrix = std::vector<std::vector<int>>(vec_row, std::vector<int>(vec_col, 0));
            for (size_t i = 0; i < infgrid_msg->cells.size(); i++) {
                pos = {infgrid_msg->cells[i].x, infgrid_msg->cells[i].y};
                MATpos = to_MATpos_m(pos, height_row, resolution);
                map_grid_matrix[MATpos[0]][MATpos[1]] = 1;
            }
        });

        // start,goal,pickをサブスクリプション
        pick_point_subscription = this->create_subscription<my_msgs::msg::PickRequest>("pick_point_topic_test", rclcpp::QoS(10), [&](const my_msgs::msg::PickRequest::SharedPtr pick_point_msg) {
            all_infmap = std::vector<std::vector<int>>(vec_row, std::vector<int>(vec_col, 0));
            all_infmap = map_grid_matrix;
            create_poses_inf(height_row, width_col, pick_point_msg, inf6, all_infmap);
            path_number = pick_point_msg->path_num;
            std::cout << "path_number:" << path_number << std::endl;
            // std::vector<int> c_matpos = to_MATpos_m({7, 4.9}, height_row, resolution);
            std::cout << "map作成完了" << std::endl;
            // std::cout << all_infmap[c_matpos[0]][c_matpos[1]] << std::endl;
        });

        // 各パスの送受信のための名前決め
        for (size_t i = 0; i < ROBOT_NUM; i++) {
            calc_path_publisher_.push_back(this->create_publisher<nav_msgs::msg::Path>("calc_path_" + std::to_string(i), rclcpp::QoS(10))); // topic name : "calc_path_0" , "calc_path_1" , ...
        }

        // 各パスの送受信のための名前決め（MAT用）
        for (size_t i = 0; i < ROBOT_NUM; i++) {
            calc_MATpath_publisher_.push_back(this->create_publisher<nav_msgs::msg::Path>("calc_MATpath_" + std::to_string(i), rclcpp::QoS(10))); // topic name : "calc_path_0" , "calc_path_1" , ...
        }

        // route_request.msgを受信
        route_request_subscription = this->create_subscription<my_msgs::msg::RouteRequest>("route_request_topic_test", rclcpp::QoS(10), [&](const my_msgs::msg::RouteRequest::SharedPtr route_request_msg) {
            auto route_calc_msg = std::make_shared<nav_msgs::msg::Path>();
            auto route_MATcalc_msg = std::make_shared<nav_msgs::msg::Path>();
            route_calc_msg->header.frame_id = "map"; // 基準座標を決めている、送るときだけ
            auto start_time = std::chrono::high_resolution_clock::now();
            int planning_time = 10 * 1000000;

            start_ = {route_request_msg->start.pose.position.x, route_request_msg->start.pose.position.y};
            goal_ = {route_request_msg->goal.pose.position.x, route_request_msg->goal.pose.position.y};
            // std::cout << "s_g" << std::endl;

            std::vector<int> ten_skip_index;
            // std::cout << "other_path.size:" << route_request_msg->other_path.size() << std::endl;
            // std::cout << route_request_msg->time_skip_vec.size() << std::endl;
            //  ここでother_path[i]を10秒後のother_pathに変える
            for (size_t i = 0; i < route_request_msg->other_path.size(); i++) {
                int skip_index_limit = (route_request_msg->time_skip_vec[i] / (sqrt_2 / resolution)) + 1; ///////////プラス1は必要ないかも

                if (route_request_msg->other_path[i].poses.size() <= skip_index_limit || route_request_msg->time_skip_vec[i] == -1) { // 10秒だから、最大0.14×70=9.8//route_request_msg->other_path[i].poses.empty() ||
                    // 空にする
                    // std::cout << "kara" << std::endl;

                    route_request_msg->other_path[i].poses.clear();
                    ten_skip_index.push_back(-1); // 考慮しないことを意味する
                }
                else {
                    // std::cout << "koryo" << std::endl;
                    // std::cout << route_request_msg->time_skip_vec[i] << std::endl;
                    double total_time = 0;
                    for (size_t j = 0; j < route_request_msg->other_path[i].poses.size() - 1; j++) {
                        double time_add = std::hypot(route_request_msg->other_path[i].poses[j + 1].pose.position.x - route_request_msg->other_path[i].poses[j].pose.position.x, route_request_msg->other_path[i].poses[j + 1].pose.position.y - route_request_msg->other_path[i].poses[j].pose.position.y);
                        // std::cout << "total_time:" << total_time << std::endl;
                        total_time = total_time + time_add;
                        if (total_time > (route_request_msg->time_skip_vec[i]) * 10) {
                            // そのときのインデックスを保持
                            // std::cout << "A" << std::endl;
                            ten_skip_index.push_back(j);
                            break;
                        }
                        //
                        if (j == route_request_msg->other_path[i].poses.size() - 2) {
                            // std::cout << "B" << std::endl;
                            route_request_msg->other_path[i].poses.clear();
                            ten_skip_index.push_back(-1); // 考慮しないことを意味する
                            break;
                        }
                    }
                }
            }

            // for (int i = 0; i < route_request_msg->other_path.size(); i++) {
            //     std::cout << i << "::" << route_request_msg->other_path[i].poses.size() << "::" << ten_skip_index[i] << std::endl;
            // }

            // std::cout << "topic3" << std::endl;
            g_foundation_list = g_layor_.renew_GfoundList(g_foundation_list_defo, height_row, width_col, resolution, robot_size, route_request_msg->other_path, ten_skip_index);
            //  start_とgoal_の障害物を解除
            Astar_map = all_infmap;
            delete_pose_inf(route_request_msg->start_num, start_, Astar_map, height_row, width_col, resolution, inf6);
            delete_pose_inf(route_request_msg->goal_num, goal_, Astar_map, height_row, width_col, resolution, inf6);

            auto path_result = a_star_.Astar_haru(start_, goal_, height_row, resolution, width_col, Astar_map, g_foundation_list, planning_time);
            std::vector<std::vector<float>> path = path_result.first;
            std::vector<std::vector<int>> MATpath = path_result.second;

            // auto end_time1 = std::chrono::high_resolution_clock::now();
            // auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end_time1 - start_time);
            // // std::cout << duration1.count() << std::endl;
            // int new_planning_time = planning_time - duration1.count();
            // std::cout << new_planning_time << std::endl;
            bool one_more_chance = true;
            int pathplanning_round = 0;

            // 手法2/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            while (one_more_chance) {
                cross_main_infother cross;
                for (size_t i = 0; i < route_request_msg->other_path.size(); i++) {
                    int other_robot_no;
                    if (i < route_request_msg->robot_no) {
                        other_robot_no = i;
                    }
                    else {
                        other_robot_no = i + 1;
                    }
                    if (!route_request_msg->other_path[i].poses.empty()) {
                        std::vector<std::vector<int>> other_infgrid = std::vector<std::vector<int>>(height_row + 1, std::vector<int>(width_col + 1, 0));
                        create_other_infgrid(route_request_msg->other_path[i], inf6, other_infgrid, height_row, width_col, ten_skip_index[i]);
                        // 交差判定して、交差する場合は、情報を取得
                        cross_judge(other_infgrid, MATpath, other_robot_no, cross, i);
                    }
                }

                cross_infmain_other cross_2;
                std::vector<std::vector<int>> main_infgrid = std::vector<std::vector<int>>(height_row + 1, std::vector<int>(width_col + 1, 0));
                create_main_infgrid(MATpath, inf6, main_infgrid, height_row, width_col);
                // 反対側の交差も取得(まとめて一気に)
                bool main_key = false;
                cross2_judge(main_infgrid, route_request_msg->other_path, ten_skip_index, cross_2, cross, count_tyohuku, main_key);

                auto end_time2 = std::chrono::high_resolution_clock::now();
                auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end_time2 - start_time);
                int new_planning_time = planning_time - duration2.count();

                if (new_planning_time < 0) {
                    std::cout << "衝突経路で妥協" << std::endl;
                    dakyo_count++;
                    break;
                }

                // 衝突判定あるなら、その部分に重みをかけて経路再生性
                if (!cross_2.vec_robot_no.empty() || main_key == true) {
                    std::cout << "衝突なので経路を変更するよ" << std::endl;
                    if (!cross_2.vec_robot_no.empty()) {
                        crossPlusCost(g_foundation_list, inf6, cross_2, route_request_msg->other_path, height_row, width_col);
                    }
                    if (main_key == true) {
                        firstMainPlusCost(g_foundation_list, inf6, MATpath, height_row, width_col);
                    }

                    path_result = a_star_.Astar_haru(start_, goal_, height_row, resolution, width_col, Astar_map, g_foundation_list, new_planning_time);
                    pathplanning_round++;

                    path = path_result.first;
                    MATpath = path_result.second;

                    if (pathplanning_round > 2) {
                        std::cout << "zissitutaiki" << std::endl;
                        path.clear();
                        MATpath.clear();
                    }

                    if (path.empty()) {
                        one_more_chance = false;
                    }
                }
                else {
                    one_more_chance = false;
                }
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // もしpathが空なら経路再生性
            if (path.empty()) {
                std::cout << "最経路生成" << std::endl;
                re_route_calc_count++;
                auto re_route_request_msg = std::make_shared<my_msgs::msg::ReRouteRequest>();
                re_route_request_msg->robot_no = route_request_msg->robot_no;
                re_route_request_msg->start_num = route_request_msg->start_num;
                re_route_request_msg->goal_num = route_request_msg->goal_num;
                re_route_request_msg->start.pose.position.x = route_request_msg->start.pose.position.x;
                re_route_request_msg->start.pose.position.y = route_request_msg->start.pose.position.y;
                re_route_request_msg->goal.pose.position.x = route_request_msg->goal.pose.position.x;
                re_route_request_msg->goal.pose.position.y = route_request_msg->goal.pose.position.y;
                re_route_request_publisher_->publish(*re_route_request_msg);
            }
            else {
                geometry_msgs::msg::PoseStamped pose;
                geometry_msgs::msg::PoseStamped MATpose;
                for (size_t i = 0; i < path.size(); i++) {
                    pose.pose.position.x = path[i][0];
                    pose.pose.position.y = path[i][1];
                    route_calc_msg->poses.push_back(pose);
                    MATpose.pose.position.x = MATpath[i][0];
                    MATpose.pose.position.y = MATpath[i][1];
                    route_MATcalc_msg->poses.push_back(MATpose);
                }
                // 該当するロボットのcalc_pathを送信
                // std::cout << "topic6" << std::endl;
                // rclcpp::sleep_for(std::chrono::seconds(6));
                auto end_time3 = std::chrono::high_resolution_clock::now();
                auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(end_time3 - start_time);
                calc_time_vec[route_request_msg->robot_no] = duration3.count() / 1000000.0f; // マイクロ秒から秒に変換
                // 結果を出力
                std::cout << "AMR" << route_request_msg->robot_no << "の経路計画時間: " << calc_time_vec[route_request_msg->robot_no] << "秒" << std::endl;
                calc_time_csv_list[route_request_msg->robot_no].push_back(calc_time_vec[route_request_msg->robot_no]);

                int totalSize = 0;
                for (int i = 0; i < ROBOT_NUM; i++) {
                    totalSize += calc_time_csv_list[i].size();
                }

                if (totalSize == path_number) {
                    writeCSV(csv_path, calc_time_csv_list);
                    std::cout << "最経路生成回数：" << re_route_calc_count << std::endl;
                    std::cout << "count_tyohuku:" << count_tyohuku << std::endl;
                    std::cout << "dakyo_count:" << dakyo_count << std::endl;

                    std::cout << "はきだしました" << std::endl;
                }

                calc_path_publisher_[route_request_msg->robot_no]->publish(*route_calc_msg);
                calc_MATpath_publisher_[route_request_msg->robot_no]->publish(*route_MATcalc_msg);
            }
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