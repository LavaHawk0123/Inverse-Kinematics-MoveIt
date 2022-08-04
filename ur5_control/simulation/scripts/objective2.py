#!/usr/bin/env python
import sys
from copy import deepcopy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import csv

def difference_z(n,o):
    l= int(n)
    f= int(o)
    diff_z = float(rows[l][2])-float(rows[f][2])
    return diff_z

def difference_y(n,o):
    l= int(n)
    f= int(o)
    diff_y = float(rows[l][1])-float(rows[f][1])
    return diff_y

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    reference_frame = 'base_link'
    manipulator_group.set_pose_reference_frame('base_link')

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
    manipulator_group.go()
    manipulator_group.stop()
    
    #initialize pushing pos (move to tag5)

    start_pose = manipulator_group.get_current_pose().pose
    waypoints=[]
    wpose = deepcopy(start_pose)
    wpose.position.x += float(rows[5][0])-0.158534-0.187
    wpose.position.y += float(rows[5][1])-0.113315+0.0093
    wpose.position.z += float(rows[5][2])-0.354882-0.053
    waypoints.append(deepcopy(wpose))
    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    # CLOSE THE GRIPPER
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = 1.396
    gripper_group.set_joint_value_target(gripper_group_variable_values)

    plan = gripper_group.plan()
    gripper_group.go(wait=True)

    #Store tags
    arr = [tag1, tag2, tag3, tag4]
    init_tag = 5

    # Set the allowable error of position (unit: meter) and attitude (unit: radians)
    manipulator_group.set_goal_position_tolerance(0.05)
    manipulator_group.set_goal_orientation_tolerance(0.05)

    # Get the name of the terminal link   
    end_effector_link = manipulator_group.get_end_effector_link()

    for i in range(len(arr)):
        goal_tag = int(arr[i])
        displacement_in_z = difference_z(goal_tag,init_tag)
        displacement_in_y = difference_y(goal_tag,init_tag)

        #  -----CODE FOR DISPLACEMENT-----
        # Get the current pose data as the starting pose of the robot arm movement
        start_pose = manipulator_group.get_current_pose().pose

        wpose = deepcopy(start_pose)

        waypoints=[]
        wpose.position.z += displacement_in_z
        waypoints.append(deepcopy(wpose))
        (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
        manipulator_group.execute(plan)

        waypoints=[]
        wpose.position.y += displacement_in_y
        waypoints.append(deepcopy(wpose))
        (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
        manipulator_group.execute(plan)

        #  -----CODE FOR PUSH-----
        waypoints=[]
        wpose.position.x +=0.031
        waypoints.append(deepcopy(wpose))
        (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
        manipulator_group.execute(plan)

        waypoints=[]
        wpose.position.x -=0.031
        waypoints.append(deepcopy(wpose))
        (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
        manipulator_group.execute(plan)

        init_tag = goal_tag

    #open gripper
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = 0
    gripper_group.set_joint_value_target(gripper_group_variable_values)

    plan = gripper_group.plan()
    gripper_group.go(wait=True)
    
    #home
    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = 0
    joint_target[1] = -2.0944
    joint_target[2] = 1.74533
    joint_target[3] = 0.34906
    joint_target[4] = 1.5708
    joint_target[5] = -1.5708
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()

if __name__=="__main__":
    rospy.init_node('objective2', anonymous=True)
    filename= rospy.get_param('~file')
    fields=[]
    rows=[]
    
    with open(filename,'r') as csvfile:
        csvreader=csv.reader(csvfile)
        fields=next(csvreader)
        for row in csvreader:
            rows.append(row)

    tag1, tag2, tag3, tag4 = rospy.get_param('~sequence').split()
    main()
