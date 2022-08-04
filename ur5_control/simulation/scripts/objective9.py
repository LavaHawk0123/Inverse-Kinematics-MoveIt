#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import copy
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main():
    #initilize moveit commander(to communicate with the move group)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")

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
    rospy.init_node("objective9", anonymous= True)
    main()
