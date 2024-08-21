#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tlab
{

class MapdSimulator : public rclcpp::Node {
private:
public:
    MapdSimulator(const rclcpp::NodeOptions& options) : MapdSimulator("", options) {}
    MapdSimulator(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("mapd_simulator_node", name_space, options) { std::cout << "hello2" << std::endl; }
};
} // namespace tlab