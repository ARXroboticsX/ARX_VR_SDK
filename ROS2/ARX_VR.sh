#!/bin/bash
source ~/.bashrc

workspace=$(pwd)

gnome-terminal -t "unity_tcp" -x bash -c "cd ${workspace};source install/setup.bash;ros2 run serial_port serial_port_node;exec bash;"

sleep 4

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source install/setup.bash;ros2 topic echo /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source install/setup.bash;ros2 topic echo /ARX_VR_R;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source install/setup.bash;ros2 topic hz /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source install/setup.bash;ros2 topic hz /ARX_VR_R;exec bash;"

