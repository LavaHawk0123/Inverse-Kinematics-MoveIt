#!/usr/bin/env python
import sys
from copy import deepcopy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupActionResult
import csv

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    reference_frame = 'base_link'
    manipulator_group.set_pose_reference_frame('base_link')

    # Set the allowable error of position (unit: meter) and attitude (unit: radians)
    manipulator_group.set_goal_position_tolerance(0.05)
    manipulator_group.set_goal_orientation_tolerance(0.05)

    # Get the name of the terminal link   
    end_effector_link = manipulator_group.get_end_effector_link()

    #move to home position
    group_variable_values = manipulator_group.get_current_joint_values()
    group_variable_values[0] = 0
    group_variable_values[1] = -2.0944
    group_variable_values[2] = 1.7453
    group_variable_values[3] = 0.3491
    group_variable_values[4] = 1.5708
    group_variable_values[5] = -1.5708
    manipulator_group.set_joint_value_target(group_variable_values)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    # Get the current pose data as the starting pose of the robot arm movement
    start_pose = manipulator_group.get_current_pose(end_effector_link).pose

    #imu
    #-----LOCATE IMU (brings gripper close to IMU)
    waypoints=[]   
    wpose = deepcopy(start_pose)           
    wpose.position.y += 0.084754
    wpose.position.z -= -0.0003868
    wpose.position.x -= -0.0196
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = 0.3839
    joint_target[1] = -1.0472
    joint_target[2] = 0.698132
    joint_target[3] = 1.9024
    joint_target[4] = 1.518
    joint_target[5] = -2.7402
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    waypoints=[]
    wpose.position.x += 0.000119
    wpose.position.y += 0.009351
    wpose.position.z -= 0.187267
    waypoints.append(deepcopy(wpose))

    wpose.orientation.w = 0.483433
    wpose.orientation.x = 0.516282
    wpose.orientation.y = 0.508302
    wpose.orientation.z = -0.491298
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = 0.4188
    joint_target[1] = -0.9424
    joint_target[2] = 1.4311
    joint_target[3] = 1.06465
    joint_target[4] = 1.518
    joint_target[5] = -1.1344
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)
    #-----Displace to the Grasping Location

    init_pose = []
    init_pose.append(0.180094)
    init_pose.append(0.206568)
    init_pose.append(0.16366)

    waypoints=[]
    wpose.position.x += float(rows[10][0])-init_pose[0]+0.06
    wpose.position.y += float(rows[10][1])-init_pose[1]-0.042
    waypoints.append(deepcopy(wpose))

    wpose.position.z += float(rows[10][2])-init_pose[2]+0.24
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    #----GRIPPER_CONTROL------(to be added)
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = 0.785398
    gripper_group.set_joint_value_target(gripper_group_variable_values)

    plan = gripper_group.plan()
    gripper_group.go(wait=True)


    #LIFT
    waypoints=[]
    wpose.position.z -= float(rows[10][2])-init_pose[2]+0.2
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()

if __name__=="__main__":
    rospy.init_node("objective3", anonymous=True)
    filename= rospy.get_param('~file')
    fields=[]
    rows=[]
    
    with open(filename,'r') as csvfile:
        csvreader=csv.reader(csvfile)
        fields=next(csvreader)
        for row in csvreader:
            rows.append(row)
    main()
