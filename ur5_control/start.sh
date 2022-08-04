#!/bin/bash -i

cd /src/ERC_2021_simulator/simulation/scripts

chmod +x objective1.py
chmod +x objective2.py
chmod +x objective3.py
chmod +x objective4.py
chmod +x final_2.py
chmod +x objective2b.py
chmod +x final_3.py
chmod +x home.py
chmod +x gripper.py


cd ../../../../..
source /catkin_ws/devel/setup.bash
cd catkin_ws
roslaunch simulation aruco_detect.launch

