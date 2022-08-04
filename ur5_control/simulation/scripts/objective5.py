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
            
    x = rows[12][0]
    y = rows[12][1]
    z = rows[12][2]

    a = rows[14][0]
    b = rows[14][1]
    c = rows[14][2]
    

    start_pose = manipulator_group.get_current_pose(end_effector_link).pose

    waypoints=[]
    wpose = deepcopy(start_pose)
    
    wpose.position.x += 0.042377
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)


    waypoints=[]
    wpose.position.y += -0.342369
    waypoints.append(deepcopy(wpose))
    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    waypoints=[]


    start_pose = manipulator_group.get_current_pose(end_effector_link).pose
    init_x=manipulator_group.get_current_pose(end_effector_link).pose.position.x 
    init_y=manipulator_group.get_current_pose(end_effector_link).pose.position.y 
    init_z=manipulator_group.get_current_pose(end_effector_link).pose.position.z

    wpose = deepcopy(start_pose)

    wpose.position.z += float(z)- init_z +0.1
    waypoints.append(deepcopy(wpose))
    wpose.position.y += float(y) - init_y - 0.02
    waypoints.append(deepcopy(wpose))
    wpose.position.x += float(x) - init_x - 0.1
    waypoints.append(deepcopy(wpose))

    wpose.orientation.w = 0.699054
    wpose.orientation.x = 0.69915
    wpose.orientation.y= -0.106061
    wpose.orientation.z = -0.106126
    waypoints.append(deepcopy(wpose))

    wpose.orientation.w = 0.686277
    wpose.orientation.x = 0.705937
    wpose.orientation.y= -0.040964
    wpose.orientation.z = -0.170291
    waypoints.append(deepcopy(wpose))

    wpose.position.x += -0.012387
    waypoints.append(deepcopy(wpose))

    wpose.position.y += -0.001192
    waypoints.append(deepcopy(wpose))

    wpose.position.z +=  -0.008773
    waypoints.append(deepcopy(wpose))
 
    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    #gripper function
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = 0.9772
    gripper_group.set_joint_value_target(gripper_group_variable_values)

    plan = gripper_group.plan()
    gripper_group.go(wait=True)

    # upward movement with wc

    start_pose = manipulator_group.get_current_pose(end_effector_link).pose
    init_x=manipulator_group.get_current_pose(end_effector_link).pose.position.x 
    init_y=manipulator_group.get_current_pose(end_effector_link).pose.position.y 
    init_z=manipulator_group.get_current_pose(end_effector_link).pose.position.z


    waypoints=[]
    wpose = deepcopy(start_pose)
    wpose.position.z += 0.311627 - init_z
    waypoints.append(deepcopy(wpose))
    wpose.position.y += -0.249542 - init_y
    waypoints.append(deepcopy(wpose))
    wpose.position.x += 0.213658 - init_x
    waypoints.append(deepcopy(wpose))


    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)    

    #place wc

    start_pose = manipulator_group.get_current_pose(end_effector_link).pose
    waypoints=[]
    wpose = deepcopy(start_pose)

    wpose.orientation.w = 0.699339
    wpose.orientation.x = 0.698887
    wpose.orientation.y= -0.107405
    wpose.orientation.z = -0.104626
    waypoints.append(deepcopy(wpose))

    wpose.orientation.w = -0.497142
    wpose.orientation.x = -0.498512
    wpose.orientation.y= 0.503366
    wpose.orientation.z = 0.500957
    waypoints.append(deepcopy(wpose))

    wpose.position.x += -0.06
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)    


    start_pose = manipulator_group.get_current_pose(end_effector_link).pose
    init_a=manipulator_group.get_current_pose(end_effector_link).pose.position.x 
    init_b=manipulator_group.get_current_pose(end_effector_link).pose.position.y 
    init_c=manipulator_group.get_current_pose(end_effector_link).pose.position.z

    waypoints=[]
    wpose = deepcopy(start_pose)
    wpose.position.z += float(c)- init_c +0.06
    waypoints.append(deepcopy(wpose))
    wpose.position.y += float(b) - init_b -0.01 + 0.036
    waypoints.append(deepcopy(wpose))
    wpose.position.x += float(a) - init_a -0.06
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan) 

    #gripper open function
    gripper_group_variable_values = gripper_group.get_current_joint_values()
    gripper_group_variable_values[0] = 0
    gripper_group.set_joint_value_target(gripper_group_variable_values)

    plan = gripper_group.plan()
    gripper_group.go(wait=True)

    start_pose = manipulator_group.get_current_pose(end_effector_link).pose
    waypoints=[]
    wpose = deepcopy(start_pose)
    wpose.position.z += 0.39
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan) 

    # home

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
    rospy.init_node("objective5", anonymous=True)
    
    filename= rospy.get_param('~file')
    fields=[]
    rows=[]
    
    with open(filename,'r') as csvfile:
        csvreader=csv.reader(csvfile)
        fields=next(csvreader)
        for row in csvreader:
            rows.append(row)

    main()