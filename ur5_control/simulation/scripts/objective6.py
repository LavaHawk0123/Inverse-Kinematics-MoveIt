#!/usr/bin/env python
import sys
from copy import deepcopy
import rospy
import csv
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform

def main():

    moveit_commander.roscpp_initialize(sys.argv)
    
    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    manipulator_group.set_pose_reference_frame('base_link')

    # Set the allowable error of position (unit: meter) and attitude (unit: radians)
    manipulator_group.set_goal_position_tolerance(0.05)
    manipulator_group.set_goal_orientation_tolerance(0.05)

    # Get the name of the terminal link   
    end_effector_link = manipulator_group.get_end_effector_link()
 
    group_variable_values = manipulator_group.get_current_joint_values()
    group_variable_values[0] = 2.6878
    group_variable_values[1] = -2.1817
    group_variable_values[2] = -0.2094
    group_variable_values[3] = -2.4958
    group_variable_values[4] = 1.5533
    group_variable_values[5] = 1.4312
    manipulator_group.set_joint_value_target(group_variable_values)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    init_pose = []
    init_pose.append(0.278694)
    init_pose.append(-0.262539)
    init_pose.append(0.401485)

    # Get the current pose data as the starting pose of the robot arm movement
    start_pose = manipulator_group.get_current_pose(end_effector_link).pose

    wpose = deepcopy(start_pose)
    waypoints=[]
    wpose.position.x += float(rows[12][0]) - 0.02289 - init_pose[0] - 0.0233 
    wpose.position.y += float(rows[12][1]) + 0.011618 - init_pose[1] - 0.045
    wpose.position.z += float(rows[12][2]) + 0.165994 - init_pose[2] + 0.01
    waypoints.append(deepcopy(wpose))
    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    # waypoints=[]
    # wpose.orientation.w = -0.331305
    # wpose.orientation.x = -0.498117
    # wpose.orientation.y = -0.493528
    # wpose.orientation.z = 0.631306
    # waypoints.append(deepcopy(wpose))

    # (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    # manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = 2.68781
    joint_target[1] = -2.0245
    joint_target[2] = -0.6806
    joint_target[3] = -2.28638
    joint_target[4] = 1.518
    joint_target[5] = 1.4311
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    waypoints=[]
    wpose.position.x += 0.02256
    wpose.position.y -= 0.0086
    wpose.position.z += 0.0131 
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    #-----SCAN----- 
    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    tag_id = save_id(sub)
    print(tag_id)
    
    waypoints=[]
    wpose.position.x -= 0.0088 
    wpose.position.y += 0.0023
    wpose.position.z -= 0.0296 
    waypoints.append(deepcopy(wpose))

    group_variable_values = manipulator_group.get_current_joint_values()
    group_variable_values[0] = 2.6878
    group_variable_values[1] = -2.14675
    group_variable_values[2] = -0.5235
    group_variable_values[3] = -2.426
    group_variable_values[4] = 1.5009
    group_variable_values[5] = 1.44862
    manipulator_group.set_joint_value_target(group_variable_values)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    # #------SCAN------ 
    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    tag_id = save_id(sub)
    print(tag_id)

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

def save_id(msg):
    tag_id = 0
    for m in msg.transforms:
        tag_id = m.fiducial_id
    
    return tag_id

if __name__=="__main__":
    rospy.init_node('objective6', anonymous=True)
    filename= rospy.get_param('~file')
    fields=[]
    rows=[]
    
    with open(filename,'r') as csvfile:
        csvreader=csv.reader(csvfile)
        fields=next(csvreader)
        for row in csvreader:
            rows.append(row)
    main()
