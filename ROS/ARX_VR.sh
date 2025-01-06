#!/bin/bash
source ~/.bashrc

workspace=$(pwd)

gnome-terminal -t "roscore" -x bash -c "cd ${workspace};source devel/setup.bash;roscore;exec bash;"
gnome-terminal -t "unity_tcp" -x bash -c "cd ${workspace};source devel/setup.bash;rosrun serial_port serial_port;exec bash;"

sleep 1

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source devel/setup.bash;rostopic echo /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source devel/setup.bash;rostopic echo /ARX_VR_R;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source devel/setup.bash;rostopic hz /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source devel/setup.bash;rostopic hz /ARX_VR_R;exec bash;"

