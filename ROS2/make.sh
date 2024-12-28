#!/bin/bash

set -e

echo "运行 colcon build..."
colcon build

source install/setup.bash

echo "编译中..."

colcon build

A_DIR="./src/ROS-TCP-Endpoint/ros_tcp_endpoint/"
B_DIR="./install/ros2/lib/python3.10/site-packages/ros_tcp_endpoint/"
# 如果目录不存在或者install编译文件夹为ros_tcp_endpoint，修改为以下行
# B_DIR="./install/ros_tcp_endpoint/lib/python3.10/site-packages/ros_tcp_endpoint/"

echo "检查目标目录..."
if [ ! -d "$B_DIR" ]; then
    echo "目标目录不存在，正在创建..."
    mkdir -p "$B_DIR"
fi

echo "拷贝文件中..."
cp -r "$A_DIR"* "$B_DIR"

echo "编译和拷贝完成。"

