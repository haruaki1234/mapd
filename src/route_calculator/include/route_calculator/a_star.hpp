#pragma once

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

struct openList_data {
    float g;
    float h;
    float f;
    int p_row;
    int p_col;
};

const int taiki_time = 10;

class Astar_Haru {
private:
    // posを行列に変換
    void to_MATpos(const std::vector<float>& pos, int height, int resolution, std::vector<int>& MATpos)
    {
        MATpos[0] = round(height - pos[1] * resolution);
        MATpos[1] = round(pos[0] * resolution);
    }

    // ヒューリスティック関数
    float heuristic(const std::vector<int>& MATpos, const std::vector<int>& MATgoal)
    {
        float h = abs(MATpos[0] - MATgoal[0]) + abs(MATpos[1] - MATgoal[1]);
        return h;
    }

    // openList更新
    void renewOpenList(int frow, int fcol, int height, int width, const std::vector<std::vector<int>>& map_grid, const std::vector<std::vector<int>>& closedList, const std::vector<std::vector<std::vector<float>>>& g_foundation_list, std::vector<std::vector<openList_data>>& openList, const std::vector<int>& MATgoal)
    {
        std::vector<int> drow = {0, 1, 1, 1, 0, -1, -1, -1};
        std::vector<int> dcol = {1, 1, 0, -1, -1, -1, 0, 1};
        int nrow;
        int ncol;
        std::vector<int> neighbor;
        float g_add;
        float tentative_g;
        float g;

        for (int i = 0; i < drow.size(); ++i) {
            nrow = frow + drow[i];
            ncol = fcol + dcol[i];

            if (!(0 <= nrow && nrow <= height && 0 <= ncol && ncol <= width) || closedList[nrow][ncol] >= 1 || map_grid[nrow][ncol] >= 1) {
                continue;
            }

            neighbor = {nrow, ncol};

            g_add = g_foundation_list[i][frow][fcol];
            tentative_g = openList[frow][fcol].g + g_add;

            if (openList[nrow][ncol].g != 0 && tentative_g > openList[nrow][ncol].g) {
                g = openList[nrow][ncol].g;
            }
            else {
                g = tentative_g;
                openList[nrow][ncol].p_row = frow; // row親更新
                openList[nrow][ncol].p_col = fcol; // col親更新
            }

            openList[nrow][ncol].g = g;                                               // g更新
            openList[nrow][ncol].h = heuristic(neighbor, MATgoal);                    // h更新
            openList[nrow][ncol].f = openList[nrow][ncol].g + openList[nrow][ncol].h; // f更新
        }
    }

    // 経路再構築
    void reConstructPath(std::vector<std::vector<int>>& MAT_path, int frow, int fcol, const std::vector<std::vector<openList_data>>& openList, int resolution, int height, std::vector<std::vector<float>>& path)
    {
        std::vector<int> MAT_path_add = {openList[frow][fcol].p_row, openList[frow][fcol].p_col};
        std::vector<std::vector<int>> attentive_path;

        while (MAT_path_add[0] != 0) {
            MAT_path.insert(MAT_path.begin(), MAT_path_add);
            frow = MAT_path_add[0];
            fcol = MAT_path_add[1];
            MAT_path_add = {openList[frow][fcol].p_row, openList[frow][fcol].p_col};
        }

        attentive_path = MAT_path;

        for (int i = 0; i < attentive_path.size(); ++i) {
            attentive_path[i][0] = height - attentive_path[i][0];
        }

        for (int i = 0; i < attentive_path.size(); ++i) {
            std::swap(attentive_path[i][0], attentive_path[i][1]);
        }

        for (int i = 0; i < attentive_path.size(); ++i) {
            path.push_back({static_cast<float>(attentive_path[i][0]) / static_cast<float>(1.0 * resolution), static_cast<float>(attentive_path[i][1]) / static_cast<float>(1.0 * resolution)});
        }
    }

public:
    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<int>>> Astar_haru(const std::vector<float>& start, const std::vector<float>& goal, int height, int resolution, int width, const std::vector<std::vector<int>>& map_grid, const std::vector<std::vector<std::vector<float>>>& g_foundation_list, int planning_time)
    {
        // std::cout << "A*内" << std::endl;

        auto start_time = std::chrono::high_resolution_clock::now();
        std::vector<int> MATstart(2);
        to_MATpos(start, height, resolution, MATstart);

        std::vector<int> MATgoal(2);
        to_MATpos(goal, height, resolution, MATgoal);

        std::vector<std::vector<openList_data>> openList(height + 1, std::vector<openList_data>(width + 1, {0, 0, 0, 0, 0}));
        std::vector<std::vector<int>> closedList(height + 1, std::vector<int>(width + 1, 0));
        // std::cout << " test_1 " << std::endl;

        openList[MATstart[0]][MATstart[1]].h = heuristic(MATstart, MATgoal); //[奥行き][行][列]
        openList[MATstart[0]][MATstart[1]].f = openList[MATstart[0]][MATstart[1]].h;

        std::vector<std::vector<float>> path;
        std::vector<std::vector<int>> MAT_path;
        std::vector<std::vector<float>> f_mat(height + 1, std::vector<float>(width + 1, 0));
        float f_min;
        int frow;
        int fcol;
        std::vector<int> MATcurrent;
        // rclcpp::sleep_for(std::chrono::seconds(9));
        // ループ
        while (true) {
            for (size_t i = 0; i < height + 1; ++i) {
                for (size_t j = 0; j < width + 1; ++j) {
                    // std::cout << openList[i][j].f << std::endl;
                    f_mat[i][j] = openList[i][j].f;
                }
            }
            f_min = std::numeric_limits<float>::max(); // 初期値をINT_MAXに設定
            for (int i = 0; i < f_mat.size(); ++i) {
                for (int j = 0; j < f_mat[i].size(); ++j) {
                    if (f_mat[i][j] != 0 && f_mat[i][j] < f_min) { // 0でないかつ現在の最小値より小さい場合に更新
                        f_min = f_mat[i][j];
                        frow = i;
                        fcol = j;
                    }
                }
            }

            MATcurrent = {frow, fcol};
            if (MATcurrent[0] == MATgoal[0] && MATcurrent[1] == MATgoal[1]) {
                MAT_path.push_back(MATgoal);
                reConstructPath(MAT_path, frow, fcol, openList, resolution, height, path);
                return std::make_pair(path, MAT_path);
            }

            closedList[frow][fcol] = 1;
            openList[frow][fcol].f = 0;
            renewOpenList(frow, fcol, height, width, map_grid, closedList, g_foundation_list, openList, MATgoal);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            if (duration.count() >= planning_time) { // 10秒 = 10 * 1000000マイクロ秒
                std::cout << "10超え" << std::endl;
                path.clear();
                MAT_path.clear();
                return std::make_pair(path, MAT_path);
            }
        }
    }
};