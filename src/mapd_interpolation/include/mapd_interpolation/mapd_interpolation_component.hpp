#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <my_msgs/msg/mapd_path.hpp>
#include <boost/math/interpolators/cubic_b_spline.hpp>

namespace tlab
{

class MapdInterpolation : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rec_path_subscription;
    rclcpp::Publisher<my_msgs::msg::MapdPath>::SharedPtr mapd_path_publisher_;
    nav_msgs::msg::Path path_query;
    nav_msgs::msg::Path path_poses;

    // 線形補間
    geometry_msgs::msg::PoseStamped linearInterpolation(const nav_msgs::msg::Path& path, float t)
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
    geometry_msgs::msg::PoseStamped splineInterpolation(const nav_msgs::msg::Path& path, float t)
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
    MapdInterpolation(const rclcpp::NodeOptions& options) : MapdInterpolation("", options) {}
    MapdInterpolation(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("mapd_interpolation_node", name_space, options)
    {
        // auto mapd_calc_msg = std::make_shared<my_msgs::msg::MapdPath>();
        // mapd_path_publisher_ = this->create_publisher<my_msgs::msg::MapdPath>("mapd_path_test", rclcpp::QoS(10));
        std::cout << "hello3" << std::endl;
        rec_path_subscription = this->create_subscription<nav_msgs::msg::Path>("path_test", rclcpp::QoS(10), [&](const nav_msgs::msg::Path::SharedPtr calc_msg) {
            std::cout << "受信できてます" << std::endl;
            auto mapd_calc_msg = std::make_shared<my_msgs::msg::MapdPath>();
            mapd_path_publisher_ = this->create_publisher<my_msgs::msg::MapdPath>("mapd_path_test", rclcpp::QoS(10));

            path_poses.poses = calc_msg->poses;

            // 重複を削除して path_query に格納
            if (!calc_msg->poses.empty()) {
                path_query.poses.push_back(calc_msg->poses[0]);

                for (size_t i = 1; i < calc_msg->poses.size(); ++i) {

                    const auto& current_pose = calc_msg->poses[i];
                    const auto& last_unique_pose = path_query.poses.back();

                    if (current_pose.pose.position.x != last_unique_pose.pose.position.x || current_pose.pose.position.y != last_unique_pose.pose.position.y) {
                        path_query.poses.push_back(current_pose);
                    }
                }
            }

            // 補間用のプログラム
            std::cout << "補間用" << std::endl;
            for (int i = 0; i < path_query.poses.size(); i++) {
                std::cout << path_query.poses[i].pose.position.x << "," << path_query.poses[i].pose.position.y << std::endl;
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position = path_query.poses[i].pose.position;
                mapd_calc_msg->query_path.poses.push_back(pose);
            }

            // 位置
            for (int i = 0; i < calc_msg->poses.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position = calc_msg->poses[i].pose.position;
                mapd_calc_msg->poses_path.poses.push_back(pose);
            }

            geometry_msgs::msg::PoseStamped result_pose = linearInterpolation(path_poses, 0.1);
            // geometry_msgs::msg::PoseStamped result_pose_2 = splineInterpolation(path_poses, 31);

            std::cout << "0.1[s]における位置:(" << result_pose.pose.position.x << "," << result_pose.pose.position.y << ")" << std::endl;
            // std::cout << "スプライン補間:(" << result_pose_2.pose.position.x << "," << result_pose_2.pose.position.y << ")" << std::endl;

            // std::cout << "size" << path_poses.poses.size() << std::endl;

            mapd_path_publisher_->publish(*mapd_calc_msg);
        });
    }
};
} // namespace tlab