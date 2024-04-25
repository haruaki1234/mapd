#include <rclcpp/rclcpp.hpp>
#include "sub_judge/sub_judge_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::SubJudge>());
    rclcpp::shutdown();
    return 0;
}