#!/bin/sh

if [ "$#" -eq 0 ]
then
echo "input: package_name"
read PKG_NAME

echo "package name: $PKG_NAME"

echo "input: node_name (e.g: \"xxx_node\")"
read NODE_NAME

echo "--node_name: $NODE_NAME"

echo "input: --dependencies (e.g: \"rclcpp std_msgs\")"
read DEPEND

echo "--dependencies: $DEPEND"

ros2 pkg create --build-type ament_cmake --dependencies $DEPEND --node-name $NODE_NAME $PKG_NAME

elif [ "$#" -eq 2 ]
then
PKG_NAME=$1
NODE_NAME=$2
echo "package name : $PKG_NAME"
echo "--node_name  : $NODE_NAME"
echo "input: --dependencies (e.g: \"rclcpp std_msgs\")"
read DEPEND

ros2 pkg create --build-type ament_cmake --dependencies $DEPEND --node-name $NODE_NAME $PKG_NAME
# ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs --node-name test_node test_pkg2
else
echo "imcorrect argument format"
fi