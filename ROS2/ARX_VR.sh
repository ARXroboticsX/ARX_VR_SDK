#!/bin/bash
source ~/.bashrc
gnome-terminal -t "VR2SDK" -x bash -c "source install/setup.sh;ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.31.137 -p ROS_TCP_PORT:=10000;exec bash;"

sleep 1

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source install/setup.sh;ros2 topic echo /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source install/setup.sh;ros2 topic echo /ARX_VR_R;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source install/setup.sh;ros2 topic hz /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source install/setup.sh;ros2 topic hz /ARX_VR_R;exec bash;"


