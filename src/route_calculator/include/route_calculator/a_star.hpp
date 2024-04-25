#pragma once

#include <iostream>
#include <vector>
#include <cmath>

struct openList_data {
    float g;
    float h;
    float f;
    int p_row;
    int p_col;
};

class Astar_Haru {
private:
    // posを行列に変換
    std::vector<int> to_MATpos(std::vector<float> pos, int height, int resolution)
    {
        float x = pos[0];
        float y = pos[1];
        int matC = round(x * resolution);
        int matR = round(height - y * resolution);
        std::vector<int> MATpos = {matR, matC};
        return MATpos;
    }

    // ヒューリスティック関数
    float heuristic(std::vector<int> MATpos, std::vector<int> MATgoal)
    {
        float h = abs(MATpos[0] - MATgoal[0]) + abs(MATpos[1] - MATgoal[1]);
        return h;
    }

    // openList更新
    std::vector<std::vector<openList_data>> renewOpenList(int frow, int fcol, int height, int width, std::vector<std::vector<int>> map_grid, std::vector<std::vector<int>> closedList, std::vector<std::vector<std::vector<float>>> g_foundation_list, std::vector<std::vector<openList_data>> openList, std::vector<int> MATgoal)
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

            if (!(0 <= nrow && nrow <= height && 0 <= ncol && ncol <= width) || closedList[nrow][ncol] >= 1) {
                if (map_grid[nrow][ncol] >= 1) {
                }
                continue;
            }
            else if (map_grid[nrow][ncol] >= 1) {
                continue;
            }

            neighbor = {nrow, ncol};

            g_add = g_foundation_list[i][frow][fcol];
            // tentative_g = openList[0][frow][fcol] + g_add;
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

            // std::cout << openList[nrow][ncol].g << std::endl;
            // std::this_thread::sleep_for(std::chrono::seconds(1));

            // std::cout << g << std::endl;
        }

        return openList;
    }

    // 経路再構築
    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<int>>> reConstructPath(std::vector<std::vector<int>> MAT_path, int frow, int fcol, std::vector<std::vector<openList_data>> openList, int resolution, int height)
    {
        std::vector<int> MAT_path_add = {openList[frow][fcol].p_row, openList[frow][fcol].p_col};
        // std::vector<std::vector<float>> path;
        std::vector<std::vector<int>> attentive_path;

        while (MAT_path_add[0] != 0) {
            MAT_path.insert(MAT_path.begin(), MAT_path_add);
            frow = MAT_path_add[0];
            fcol = MAT_path_add[1];
            MAT_path_add = {openList[frow][fcol].p_row, openList[frow][fcol].p_col};
        }

        std::vector<std::vector<float>> path(MAT_path.size(), std::vector<float>(2, 0));

        attentive_path = MAT_path;
        std::cout << " test_7.6 " << std::endl;

        for (int i = 0; i < attentive_path.size(); ++i) {
            attentive_path[i][0] = height - attentive_path[i][0];
        }
        std::cout << " test_7.67 " << std::endl;

        for (int i = 0; i < attentive_path.size(); ++i) {
            std::swap(attentive_path[i][0], attentive_path[i][1]);
        }
        std::cout << " test_7.7 " << std::endl;

        for (int i = 0; i < attentive_path.size(); ++i) {
            // path[i][0] = attentive_path[i][0] * (1.0 / resolution);
            // path[i][1] = attentive_path[i][1] * (1.0 / resolution);
            path[i][0] = attentive_path[i][0] / (1.0 * resolution);
            path[i][1] = attentive_path[i][1] / (1.0 * resolution);
        }

        return std::make_pair(path, MAT_path);
    }

public:
    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<int>>> Astar_haru(std::vector<float> start, std::vector<float> goal, int height, int resolution, int width, std::vector<std::vector<int>> map_grid, std::vector<std::vector<std::vector<float>>> g_foundation_list)
    {
        std::vector<int> MATstart = to_MATpos(start, height, resolution);
        std::vector<int> MATgoal = to_MATpos(goal, height, resolution);
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
        // std::cout << frow << std::endl;
        // std::cout << fcol << std::endl;

        // ループ
        while (true) {
            // std::cout << " test_3 " << std::endl;
            for (size_t i = 0; i < height + 1; ++i) {
                for (size_t j = 0; j < width + 1; ++j) {
                    // std::cout << openList[i][j].f << std::endl;
                    f_mat[i][j] = openList[i][j].f;
                }
            }
            // std::cout << " test_4 " << std::endl;
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

            // std::cout << frow << std::endl;
            // std::cout << fcol << std::endl;

            // std::cout << f_min << std::endl;

            // if (f_min == 0) {
            //     return std::make_pair(path, MAT_path);
            // }
            // std::cout << " test_6 " << std::endl;

            MATcurrent = {frow, fcol};
            // std::cout << MATgoal[0] << std::endl;
            // std::cout << MATgoal[1] << std::endl;
            if (MATcurrent[0] == MATgoal[0] && MATcurrent[1] == MATgoal[1]) {
                MAT_path.push_back(MATgoal);
                std::cout << " test_7 " << std::endl;
                auto path_result = reConstructPath(MAT_path, frow, fcol, openList, resolution, height);
                std::cout << " test_8 " << std::endl;
                path = path_result.first;
                MAT_path = path_result.second;
                return std::make_pair(path, MAT_path);
            }

            closedList[frow][fcol] = 1;
            openList[frow][fcol].f = 0;
            openList = renewOpenList(frow, fcol, height, width, map_grid, closedList, g_foundation_list, openList, MATgoal);
        }
    }
};