#!/bin/bash

set -e

echo "Running colcon build..."
colcon build

source install/setup.bash

colcon build

A_DIR="./src/ROS-TCP-Endpoint/ros_tcp_endpoint/"
B_DIR="./install/ros_tcp_endpoint/lib/python3.10/site-packages/ros_tcp_endpoint/"

echo "拷贝文件中"
cp -r "$A_DIR"* "$B_DIR"

echo "编译完成"

