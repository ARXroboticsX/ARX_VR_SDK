#!/bin/bash
source ~/.bashrc
gnome-terminal -t "unity_tcp" -x bash -c "source devel/setup.bash;roslaunch ros_tcp_endpoint endpoint.launch;exec bash;"

sleep 1

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source devel/setup.bash;rostopic echo /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source devel/setup.bash;rostopic echo /ARX_VR_R;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source devel/setup.bash;rostopic hz /ARX_VR_L;exec bash;"

gnome-terminal -t "arx5_pos_cmd" -x bash -c "source devel/setup.bash;rostopic hz /ARX_VR_R;exec bash;"

