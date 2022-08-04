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

    # Get the current pose data as the starting pose of the robot arm movement
    start_pose = manipulator_group.get_current_pose(end_effector_link).pose

    waypoints=[]
    wpose = deepcopy(start_pose)
    wpose.position.x -= 0.052943 
    wpose.position.y -= 0.075394
    wpose.position.z += 0.18358
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = 0.3316
    joint_target[1] = -1.9373
    joint_target[2] = 1.5
    joint_target[3] = 0.4363
    joint_target[4] = 1.466
    joint_target[5] = -3.1415
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    waypoints=[]
    wpose.position.x += 0.052993
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    # determining the z co-ordinate where the imu needs to be placed
    init_pose = []
    init_pose.append(0.261911)
    init_pose.append(0.196186)
    init_pose.append(0.347254)

    waypoints=[]
    wpose.position.z += float(rows[5][2])+0.05349-init_pose[2]
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    #Angle of Rotation
    group_variable_values = manipulator_group.get_current_joint_values()
    group_variable_values[5] = -3.14159-((ang*3.14159)/180.0)
    manipulator_group.set_joint_value_target(group_variable_values)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    # Get the current pose data as the starting pose of the robot arm movement
    start_pose = manipulator_group.get_current_pose(end_effector_link).pose
    wpose = deepcopy(start_pose)
    #stick
    waypoints=[]
    wpose.position.x += 0.052993
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    #Gripper Open
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = 0
    gripper_group.set_joint_value_target(gripper_group_variable_values)

    plan = gripper_group.plan()
    gripper_group.go(wait=True)

    #move back from panel

    waypoints=[]
    wpose.position.x -= 0.105986
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    #Home
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
    rospy.init_node("objective4", anonymous=True)

    filename= rospy.get_param('~file')
    fields=[]
    rows=[]
    
    with open(filename,'r') as csvfile:
        csvreader=csv.reader(csvfile)
        fields=next(csvreader)
        for row in csvreader:
            rows.append(row)

    ang = rospy.get_param('~angle')
    main()
