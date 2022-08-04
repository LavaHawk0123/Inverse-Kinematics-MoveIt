#!/usr/bin/env python
import rospy
import sys
import moveit_commander
import moveit_msgs.msg

def main():

    moveit_commander.roscpp_initialize(sys.argv)
    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")

    # construct a message
    joint_constraint = moveit_msgs.msg.JointConstraint()
    joint_constraint.joint_name = manipulator_group.get_joints()[4]
    joint_constraint.tolerance_above =  2.30383
    joint_constraint.tolerance_below = -3.1415
    joint_constraint_list = []
    joint_constraint_list.append(joint_constraint)

    constraint_list = moveit_msgs.msg.Constraints()
    constraint_list.name = 'wrist_2_link joint '
    constraint_list.joint_constraints = joint_constraint_list
    manipulator_group.set_path_constraints

if __name__=="__main__":
    rospy.init_node("w2lconstraint", anonymous= True)
    main()
