# Inverse-Kinematics-MoveIt
The repository contains a driver for configuring Inverse Kinematics on a UR5 Robotic Arm and making it perform several tasks on an Inspection Panel
<br>
Simulation Software: Gazebo
Framework: ROS
Dependencies used:
<ol><li>MoveIt
<li>math
<li>Numpy
<li>time
<li>rospy</ol>

Sensors used:
1) Stereocamera 
2) Imu
3) UR5 Robotic Arm

<h2> Installation Instructions</h3>
If you don't have a catkin workspace,open a terminal and run

```
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make

```
After creating a workspace, open a terminal and execute

```
cd catkin_ws/src
git pull https://github.com/LavaHawk0123/Inverse-Kinematics-MoveIt.git
cd /simulation/scripts

chmod +x objective1.py
chmod +x objective2.py
chmod +x objective3.py
chmod +x objective4.py
chmod +x final_2.py
chmod +x objective2b.py
chmod +x final_3.py
chmod +x home.py
chmod +x gripper.py
```

Now that all the files are executable, run : 
```
roslaunch simulation aruco_detect.launch
```

<h3> Simulation output</h3>

![ur5_sim](https://user-images.githubusercontent.com/75236655/182920688-79b243a7-e8d8-4fbb-8145-672ec7efcd69.gif)
