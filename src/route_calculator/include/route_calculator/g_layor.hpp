#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/path.hpp>

// struct G_cost {
//     std::vector<float> g_foundation = std::vector<float>(8);
//     int robot_num;
// };

class G_Layor {
private:
    // posを行列に変換
    std::vector<float> to_MATpos_g(std::vector<float> pos, int height, int resolution)
    {
        float x = pos[0];
        float y = pos[1];
        float matC = x * resolution;
        float matR = height - y * resolution;
        std::vector<float> MATpos = {matR, matC};
        return MATpos;
    }

public:
    // g_foundationList初期構成の作成
    std::vector<std::vector<std::vector<float>>> GfoundList_createIni(int height, int width, float sqrt_2)
    {
        std::vector<float> g_foundation = {1, sqrt_2, 1, sqrt_2, 1, sqrt_2, 1, sqrt_2};
        std::vector<std::vector<std::vector<float>>> g_foundation_list(g_foundation.size(), std::vector<std::vector<float>>(height + 1, std::vector<float>(width + 1, 0)));

        for (int i = 0; i < g_foundation.size(); ++i) {
            for (int row = 0; row < (height + 1); ++row) {
                for (int col = 0; col < (width + 1); ++col) {
                    g_foundation_list[i][row][col] = g_foundation[i];
                }
            }
        }
        return g_foundation_list;
    }

    std::vector<std::vector<std::vector<float>>> renew_GfoundList(const std::vector<std::vector<std::vector<float>>>& g_foundation_list_defo, int height_row, int width_col, int resolution, float infsize, const std::vector<nav_msgs::msg::Path>& other_paths, const std::vector<int>& ten_skip_index)
    {
        int inf6 = infsize * resolution;
        std::vector<std::vector<float>> attentive_map = std::vector<std::vector<float>>(height_row + 2, std::vector<float>(width_col + 2, 0));
        for (int i = 0; i < other_paths.size(); i++) {
            if (!other_paths[i].poses.empty()) {
                for (size_t j = ten_skip_index[i]; j < other_paths[i].poses.size(); j++) {
                    std::vector<float> MATpos = {other_paths[i].poses[j].pose.position.x, other_paths[i].poses[j].pose.position.y};
                    // std::vector<float> MATpos = to_MATpos_g(pos, height_row, resolution);
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
        }
        std::vector<std::vector<float>> calc_map = std::vector<std::vector<float>>(height_row + 3, std::vector<float>(width_col + 3, 0));
        std::vector<std::vector<std::vector<float>>> real_g_foundation_list = g_foundation_list_defo;
        for (int i = 0; i < height_row + 1; i++) {
            for (int j = 0; j < width_col + 1; j++) {
                calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];
                if (calc_map[i + 1][j + 1] != 0) {
                    for (int k = 0; k < 8; k++) {
                        real_g_foundation_list[k][i][j] += 0.1;
                    }
                }
            }
        }

        return real_g_foundation_list;
    }

    // void crossPlusCost(std::vector<std::vector<std::vector<float>>>& g_foundation_list, int inf6, const cross_infmain_other& cross_2, const std::vector<nav_msgs::msg::Path>& other_paths, int height_row, int width_col)
    // {
    //     std::vector<std::vector<float>> attentive_map = std::vector<std::vector<float>>(height_row + 2, std::vector<float>(width_col + 2, 0));
    //     for (int i = 0; i < cross_2.vec_robot_no.size(); i++) {
    //         for (int j = cross_2.vec_cross_start_index[i]; i < cross_2.vec_cross_end_index[i]; i++) {
    //             std::vector<float> MATpos = {other_paths[cross_2.vec_robot_no[i]].poses[j].pose.position.x, other_paths[cross_2.vec_robot_no[i]].poses[j].pose.position.y};
    //             int min_row = MATpos[0] - inf6;
    //             int max_row = MATpos[0] + inf6;
    //             int min_col = MATpos[1] - inf6;
    //             int max_col = MATpos[1] + inf6;
    //             if (min_row < 0) {
    //                 min_row = 0;
    //             }
    //             if (min_col < 0) {
    //                 min_col = 0;
    //             }
    //             if (max_row > height_row) {
    //                 max_row = height_row;
    //             }
    //             if (max_col > width_col) {
    //                 max_col = width_col;
    //             }
    //             attentive_map[min_row][min_col] += 1;
    //             attentive_map[max_row + 1][max_col + 1] += 1;
    //             attentive_map[max_row + 1][min_col] += -1;
    //             attentive_map[min_row][max_col + 1] += -1;
    //         }
    //     }
    //     std::vector<std::vector<float>> calc_map = std::vector<std::vector<float>>(height_row + 3, std::vector<float>(width_col + 3, 0));
    //     for (int i = 0; i < height_row + 1; i++) {
    //         for (int j = 0; j < width_col + 1; j++) {
    //             calc_map[i + 1][j + 1] = attentive_map[i][j] + calc_map[i][j + 1] + calc_map[i + 1][j] - calc_map[i][j];
    //             if (calc_map[i + 1][j + 1] != 0) {
    //                 for (int k = 0; k < 8; k++) {
    //                     g_foundation_list[k][i][j] += 10;
    //                 }
    //             }
    //         }
    //     }
    // }

    // 4次元のg_foundationList初期構成の作成
    std::vector<std::vector<std::vector<std::vector<float>>>> GfoundList_createIni_4D(int height, int width, float sqrt_2, int ROBOT_NUM)
    {
        std::vector<float> g_foundation = {1, sqrt_2, 1, sqrt_2, 1, sqrt_2, 1, sqrt_2};
        std::vector<std::vector<std::vector<std::vector<float>>>> g_foundation_list(ROBOT_NUM, std::vector<std::vector<std::vector<float>>>(g_foundation.size(), std::vector<std::vector<float>>(height + 1, std::vector<float>(width + 1, 0))));
        for (int ro = 0; ro < ROBOT_NUM; ro++) {
            for (int i = 0; i < g_foundation.size(); ++i) {
                for (int row = 0; row < (height + 1); ++row) {
                    for (int col = 0; col < (width + 1); ++col) {
                        g_foundation_list[ro][i][row][col] = g_foundation[i];
                    }
                }
            }
        }
        return g_foundation_list;
    }

    // g_foundationListの他ロボット重み増(4D)
    std::vector<std::vector<std::vector<std::vector<float>>>> GfoundList_other_robot_obstacle_4D(std::vector<std::vector<std::vector<std::vector<float>>>> g_foundation_list, nav_msgs::msg::Path other_paths_, int resolution, float robot_size, int ro)
    {
        int infsize = robot_size * resolution;
        int height_row = g_foundation_list[0][0].size();
        int width_col = g_foundation_list[0][0][0].size();
        std::vector<float> pos;
        std::vector<float> MATpos;
        for (int i = 0; i < other_paths_.poses.size(); i++) {
            pos = {other_paths_.poses[i].pose.position.x, other_paths_.poses[i].pose.position.y};
            MATpos = to_MATpos_g(pos, height_row, resolution);
            int min_row = MATpos[0] - infsize;
            int max_row = MATpos[0] + infsize;
            int min_col = MATpos[1] - infsize;
            int max_col = MATpos[1] + infsize;
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

            for (int j = min_row; j <= max_row; j++) {
                for (int k = min_col; k <= max_col; k++) {
                    for (int q = 0; q < g_foundation_list.size(); q++) {
                        g_foundation_list[ro][q][j][k] = 1000;
                    }
                }
            }
        }
        return g_foundation_list;
    }

    // 毎回roのとこだけ初期化
    std::vector<std::vector<std::vector<std::vector<float>>>> G_parton_Ini(std::vector<std::vector<std::vector<std::vector<float>>>> g_foundation_list, int ro, int height, int width, float sqrt_2)
    {
        std::vector<float> g_foundation = {1, sqrt_2, 1, sqrt_2, 1, sqrt_2, 1, sqrt_2};
        for (int i = 0; i < g_foundation.size(); ++i) {
            for (int row = 0; row < (height + 1); ++row) {
                for (int col = 0; col < (width + 1); ++col) {
                    g_foundation_list[ro][i][row][col] = g_foundation[i];
                }
            }
        }
        return g_foundation_list;
    }
};
