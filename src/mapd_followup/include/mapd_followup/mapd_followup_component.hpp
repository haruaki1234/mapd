#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <my_msgs/msg/mapd_path.hpp>
#include <random>
#include <boost/math/interpolators/cubic_b_spline.hpp>

namespace tlab
{

class MapdFollowup : public rclcpp::Node {
private:
    rclcpp::Subscription<my_msgs::msg::MapdPath>::SharedPtr mapd_path_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    nav_msgs::msg::Path path_query;
    nav_msgs::msg::Path path_poses_2;

    rclcpp::TimerBase::SharedPtr timer_;

    bool rec = false;
    float elapsed_index = 0.1;

    std::vector<float> current_pos = std::vector<float>(2, 0);
    std::vector<float> pre_current_pos = {3.5, 3.5};
    geometry_msgs::msg::PoseStamped pos_ref;
    std::vector<float> v_in = std::vector<float>(2, 0);
    std::vector<float> error = std::vector<float>(2, 0);
    float dt = 0.1;
    float Kp = 1;

    // ランダムに位置誤差を生成する関数
    std::vector<float> generateRandomErrors()
    {
        std::vector<float> error = std::vector<float>(2, 0);

        // ランダム数生成器
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-0.02, 0.02);

        // ベクターにランダムな値を代入
        for (auto& e : error) {
            e = dis(gen);
        }

        return error;
    }

    //  線形補間
    geometry_msgs::msg::PoseStamped linearInterpolation_2(const nav_msgs::msg::Path& path, float t)
    {
        geometry_msgs::msg::PoseStamped result;

        if (t <= path.poses.front().header.stamp.sec) {
            return path.poses.front();
        }
        if (t >= path.poses.back().header.stamp.sec) {
            return path.poses.back();
        }

        for (size_t i = 1; i < path.poses.size(); ++i) {
            if (t <= path.poses[i].header.stamp.sec) {
                float t0 = path.poses[i - 1].header.stamp.sec;
                float t1 = path.poses[i].header.stamp.sec;

                float x0 = path.poses[i - 1].pose.position.x;
                float x1 = path.poses[i].pose.position.x;
                float y0 = path.poses[i - 1].pose.position.y;
                float y1 = path.poses[i].pose.position.y;

                float factor = (t - t0) / (t1 - t0);

                result.pose.position.x = x0 + factor * (x1 - x0);
                result.pose.position.y = y0 + factor * (y1 - y0);
                result.pose.position.z = path.poses[i - 1].pose.position.z + factor * (path.poses[i].pose.position.z - path.poses[i - 1].pose.position.z);

                // 補間の時間を設定
                result.header.stamp.sec = t;
                result.header.stamp.nanosec = 0;

                return result;
            }
        }

        return path.poses.back(); // 予期しない場合のための安全策
    }

    // スプライン補間
    geometry_msgs::msg::PoseStamped splineInterpolation_2(const nav_msgs::msg::Path& path, float t)
    {
        geometry_msgs::msg::PoseStamped result;
        std::vector<float> times;
        std::vector<float> xs, ys, zs;

        for (const auto& pose_stamped : path.poses) {
            times.push_back(pose_stamped.header.stamp.sec);
            xs.push_back(pose_stamped.pose.position.x);
            ys.push_back(pose_stamped.pose.position.y);
            zs.push_back(pose_stamped.pose.position.z);
        }

        boost::math::cubic_b_spline<float> spline_x(xs.begin(), xs.end(), times.front(), (times.back() - times.front()) / (times.size() - 1));
        boost::math::cubic_b_spline<float> spline_y(ys.begin(), ys.end(), times.front(), (times.back() - times.front()) / (times.size() - 1));
        boost::math::cubic_b_spline<float> spline_z(zs.begin(), zs.end(), times.front(), (times.back() - times.front()) / (times.size() - 1));

        result.pose.position.x = spline_x(t);
        result.pose.position.y = spline_y(t);
        result.pose.position.z = spline_z(t);

        // 補間の時間を設定
        result.header.stamp.sec = t;
        result.header.stamp.nanosec = 0;

        return result;
    }

public:
    MapdFollowup(const rclcpp::NodeOptions& options) : MapdFollowup("", options) {}
    MapdFollowup(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("mapd_followup_node", name_space, options)
    {
        using namespace std::chrono_literals;
        // auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        // pose_msg->header.frame_id = "map2"; // 基準座標を決めている、送るときだけ
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mapd_pose_test", rclcpp::QoS(10));
        std::cout << "hello4" << std::endl;
        mapd_path_subscription = this->create_subscription<my_msgs::msg::MapdPath>("mapd_path_test", rclcpp::QoS(10), [&](const my_msgs::msg::MapdPath::SharedPtr mapd_calc_msg) {
            std::cout << "さらに受信" << std::endl;
            for (int i = 0; i < mapd_calc_msg->query_path.poses.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position = mapd_calc_msg->query_path.poses[i].pose.position;
                path_query.poses.push_back(pose);
            }

            for (int i = 0; i < mapd_calc_msg->poses_path.poses.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position = mapd_calc_msg->poses_path.poses[i].pose.position;
                pose.header.stamp.sec = i;
                path_poses_2.poses.push_back(pose);
            }

            std::cout << "path_query.poses.size():" << path_query.poses.size() << std::endl;
            std::cout << "path_poses.poses.size():" << path_poses_2.poses.size() << std::endl;
            rec = true;
        });

        timer_ = this->create_wall_timer(100ms, [&]() {
            if (rec == true) {
                auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
                pose_msg->header.frame_id = "map2"; // 基準座標を決めている、送るときだけ

                geometry_msgs::msg::PoseStamped result_pose = linearInterpolation_2(path_poses_2, 1);
                pos_ref = linearInterpolation_2(path_poses_2, elapsed_index);

                v_in[0] = Kp * (pos_ref.pose.position.x - pre_current_pos[0]) / dt;
                v_in[1] = Kp * (pos_ref.pose.position.y - pre_current_pos[1]) / dt;

                error = generateRandomErrors();

                current_pos[0] = pre_current_pos[0] + v_in[0] * dt; //+ error[0]
                current_pos[1] = pre_current_pos[1] + v_in[1] * dt; //+ error[1]

                pose_msg->pose.position.x = current_pos[0];
                pose_msg->pose.position.y = current_pos[1];

                std::cout << "pos_ref.pose.position.x:" << pos_ref.pose.position.x << ",pos_ref.pose.position.y:" << pos_ref.pose.position.y << std::endl;
                std::cout << "pre_current_pos[0]:" << pre_current_pos[0] << ",pre_current_pos[1]:" << pre_current_pos[1] << std::endl;
                std::cout << "current_pos[0]:" << current_pos[0] << ",current_pos[1]:" << current_pos[1] << std::endl;
                std::cout << "v_in[0]:" << v_in[0] << ",v_in[1]:" << v_in[1] << std::endl;

                // pose_msg->pose.position = path_poses.poses[index].pose.position;

                pose_publisher_->publish(*pose_msg);
                pre_current_pos = current_pos;
                elapsed_index += 0.1;
                if (elapsed_index == path_poses_2.poses.size() - 1) {
                    elapsed_index = 0;
                    rec = false;
                }
            }
        });
    }
};
} // namespace tlab