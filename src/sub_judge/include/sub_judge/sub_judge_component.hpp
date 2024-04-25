#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tlab
{

class SubJudge : public rclcpp::Node {
private:
    rclcpp::Node::SharedPtr node; // ノードへのポインターを定義
public:
    SubJudge(const rclcpp::NodeOptions& options) : SubJudge("", options) {}
    SubJudge(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("sub_judge_node", name_space, options)
    {
        // using namespace std::chrono_literals;
        // node = rclcpp::Node::make_shared("minimal_node");
        // rclcpp::Rate loop_rate(1);

        //  // 3回のループを行い、ログを出力
        // for(int i=0 ; i<3 ; i++){
        //     RCLCPP_INFO(get_logger(), "loop:%d", i); // MinimalTimerクラス内で直接get_logger()を使用可能
        //     loop_rate.sleep();
        // }
    }
};

} // namespace tlab