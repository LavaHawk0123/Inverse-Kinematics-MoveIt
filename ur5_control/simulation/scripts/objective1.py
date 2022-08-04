#!/usr/bin/env python
import sys
from copy import deepcopy
import numpy as np
import pandas as pd
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gazebo_msgs.srv import GetModelState
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform

global sub

class tag_info():
    aruco_id = None 
    position_x = None 
    position_y = None 
    position_z = None 

list = []

for i in range(15):
    list.append(tag_info())

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    reference_frame = 'base_link'
    manipulator_group.set_pose_reference_frame('base_link')

    # Set the allowable error of position (unit: meter) and attitude (unit: radians)
    manipulator_group.set_goal_position_tolerance(0.05)
    manipulator_group.set_goal_orientation_tolerance(0.05)

    # Get the name of the terminal link   
    end_effector_link = manipulator_group.get_end_effector_link()

    # Get the current pose data as the starting pose of the robot arm movement
    start_pose = manipulator_group.get_current_pose().pose
    waypoints=[]

    #button 1
    
    wpose = deepcopy(start_pose)
    wpose.position.x += 0.15
    wpose.position.z += 0.031
    wpose.position.y -= 0.03
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_1 = np.array([[ 0.0767400281,  -0.000384013684,  0.997051694, 0.30002],
                            [0.0000780394847, -0.999999920, -0.000391155831, 0.082311],
                            [0.997051765,  0.000107826418, -0.0767388215, 0.46828],
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_1, matrix)
    
    list[0].aruco_id = A_id
    list[0].position_x = aruco[0,3]
    list[0].position_y = aruco[1,3]
    list[0].position_z = aruco[2,3]

    print("ID:" + str(list[0].aruco_id), "X:" + str(list[0].position_x), "Y:" + str(list[0].position_y), "Z:" + str(list[0].position_z))
    
    # button 2
    waypoints=[]
    wpose.position.y -=0.086
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_2 = np.array([[ 0.0767165600, -0.000371527502, 0.997052568, 0.300065], 
                            [0.000133740901, -0.999999918, -0.000382916165, -0.00377095], 
                            [0.997052629, 0.000162722873, -0.0767170681, 0.468332], 
                            [0, 0, 0, 1]] )

    aruco = np.matmul(waypoint_2, matrix)

    list[1].aruco_id = A_id
    list[1].position_x = aruco[0,3]
    list[1].position_y = aruco[1,3]
    list[1].position_z = aruco[2,3]

    print("ID:" + str(list[1].aruco_id), "X:" + str(list[1].position_x), "Y:" + str(list[1].position_y), "Z:" + str(list[1].position_z))

    # button 3
    waypoints=[]
    wpose.position.y -=0.086
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)
      
    waypoint_3 = np.array([[ 0.0767370947, -0.000389849141,  0.997051802, 0.29998],
                            [0.0000782842913, -0.999999918, -0.000397027082, -0.089664],
                            [0.997051875,  0.000108519957, -0.0767361022, 0.46829],
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_3, matrix)
    
    list[2].aruco_id = A_id
    list[2].position_x = aruco[0,3]
    list[2].position_y = aruco[1,3]
    list[2].position_z = aruco[2,3]

    print("ID:" + str(list[2].aruco_id), "X:" + str(list[2].position_x), "Y:" + str(list[2].position_y), "Z:" + str(list[2].position_z))
    
    # button 6
    waypoints=[]
    wpose.position.z -=0.15
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_4 = np.array([[ 0.0767370948, -0.000390094053,  0.997051802, 0.30003],
                            [0.0000782269037, -0.999999918, -0.000397268301, -0.089693],
                            [0.997051875,  0.000108481249, -0.0767361022, 0.31828],
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_4, matrix)
    
    list[3].aruco_id = A_id
    list[3].position_x = aruco[0,3]
    list[3].position_y = aruco[1,3]
    list[3].position_z = aruco[2,3]

    print("ID:" + str(list[3].aruco_id), "X:" + str(list[3].position_x), "Y:" + str(list[3].position_y), "Z:" + str(list[3].position_z))

    # button 5
    waypoints=[]
    wpose.position.y += 0.086
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_5 = np.array([[ 0.0767370939, -0.000387111144, 0.997050335, 0.3], 
                            [0.0000779206632, -0.999999919, -0.000394253160, -0.00365545], 
                            [0.997050407,  0.000107945108, -0.0767388207, 0.31831], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_5, matrix)
    
    list[4].aruco_id = A_id
    list[4].position_x = aruco[0,3]
    list[4].position_y = aruco[1,3]
    list[4].position_z = aruco[2,3]

    print("ID:" + str(list[4].aruco_id), "X:" + str(list[4].position_x), "Y:" + str(list[4].position_y), "Z:" + str(list[4].position_z))

    # button 4
    waypoints=[]
    wpose.position.y += 0.086
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_6 = np.array([[ 0.0767400281, -0.000383960789, 0.997051694, 0.300006], 
                            [0.0000777635337, -0.999999921, -0.000391081541, 0.0823282], 
                            [0.997051765, 0.000107545580, -0.0767388216, 0.318264], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_6, matrix)
    
    list[5].aruco_id = A_id
    list[5].position_x = aruco[0,3]
    list[5].position_y = aruco[1,3]
    list[5].position_z = aruco[2,3]

    print("ID:" + str(list[5].aruco_id), "X:" + str(list[5].position_x), "Y:" + str(list[5].position_y), "Z:" + str(list[5].position_z))

    # button 7
    waypoints=[]
    wpose.position.z -=0.15
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_7 = np.array([[ 0.0767400281, -0.000384172798, 0.997050227, 0.300002], 
                            [0.0000775617155, -0.999999920, -0.000391278802, 0.0823496], 
                            [0.997050298, 0.000107360058, -0.0767415393, 0.168267], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_7, matrix)
    
    list[6].aruco_id = A_id
    list[6].position_x = aruco[0,3]
    list[6].position_y = aruco[1,3]
    list[6].position_z = aruco[2,3]

    print("ID:" + str(list[6].aruco_id), "X:" + str(list[6].position_x), "Y:" + str(list[6].position_y), "Z:" + str(list[6].position_z))

    # button 8
    waypoints=[]
    wpose.position.y -=0.086
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)
    
    waypoint_8 = np.array([[ 0.0766989435, -0.000360876034, 0.997054688, 0.299917], 
                            [0.000117812094, -0.999999924, -0.000371004897, -0.00354107], 
                            [0.997054746, 0.000145920566, -0.0766980508, 0.168274], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_8, matrix)
    
    list[7].aruco_id = A_id
    list[7].position_x = aruco[0,3]
    list[7].position_y = aruco[1,3]
    list[7].position_z = aruco[2,3]

    print("ID:" + str(list[7].aruco_id), "X:" + str(list[7].position_x), "Y:" + str(list[7].position_y), "Z:" + str(list[7].position_z))

    # button 9
    waypoints=[]
    wpose.position.y -=0.086
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_9 = np.array([[ 0.0767194849, -0.000367705723, 0.997052458, 0.299923], 
                            [0.0000993175609, -0.999999924, -0.000376434797, -0.0895417], 
                            [0.997052521, 0.000127904791, -0.0767197960, 0.168294], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_9, matrix)

    list[8].aruco_id = A_id
    list[8].position_x = aruco[0,3]
    list[8].position_y = aruco[1,3]
    list[8].position_z = aruco[2,3]

    print("ID:" + str(list[8].aruco_id), "X:" + str(list[8].position_x), "Y:" + str(list[8].position_y), "Z:" + str(list[8].position_z))

    # Left Panel
    waypoints=[]
    wpose.position.y +=0.2345
    wpose.position.z +=0.2453
    waypoints.append(deepcopy(wpose))
    wpose.position.x -=0.115098
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = -0.069
    joint_target[1] = -1.9722
    joint_target[2] = 1.7802
    joint_target[3] = 0.1919
    joint_target[4] = 0.9773
    joint_target[5] = -1.5708
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)
    
    waypoint_10 = np.array([[ 0.150536686, 0.0867305758, 0.984791858, 0.193579], 
                            [0.500116745, -0.865957869, -0.000184077432, 0.144023], 
                            [0.852772207, 0.492538932, -0.173735081, 0.4142], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_10, matrix)
    
    list[9].aruco_id = A_id
    list[9].position_x = aruco[0,3]
    list[9].position_y = aruco[1,3]
    list[9].position_z = aruco[2,3]

    print("ID:" + str(list[9].aruco_id), "X:" + str(list[9].position_x), "Y:" + str(list[9].position_y), "Z:" + str(list[9].position_z))

    # imu 
    waypoints=[]              
    wpose.position.y += 0.084754
    wpose.position.z -= -0.0003868
    wpose.position.x -= -0.0196
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = 0.4363
    joint_target[1] = -0.6632
    joint_target[2] = 0.1047
    joint_target[3] = 2.059
    joint_target[4] = 1.501
    joint_target[5] = -2.6529
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_11 = np.array([[ 0.0216536290, 0.991998431, -0.124378726, 0.216821], 
                            [0.999759780, -0.0219044114, -0.000647640648, 0.316901], 
                            [-0.00336689147, -0.124334834, -0.992234605, 0.333462], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_11, matrix)
    
    list[10].aruco_id = A_id
    list[10].position_x = aruco[0,3]
    list[10].position_y = aruco[1,3]
    list[10].position_z = aruco[2,3]

    print("ID:" + str(list[10].aruco_id), "X:" + str(list[10].position_x), "Y:" + str(list[10].position_y), "Z:" + str(list[10].position_z))

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

    waypoints=[]
    start_pose = manipulator_group.get_current_pose().pose
    wpose = deepcopy(start_pose)

    # right panel
    # big red box

    wpose.position.x += 0.039425
    wpose.position.y -= 0.05195
    waypoints.append(deepcopy(wpose))

    wpose.position.x += 0.018
    wpose.position.y -= 0.1871
    waypoints.append(deepcopy(wpose))

    wpose.position.x -= 0.043
    waypoints.append(deepcopy(wpose))

    wpose.position.y -= 0.0433
    wpose.position.x -= 0.0159
    waypoints.append(deepcopy(wpose))

    wpose.position.z -= 0.24298
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = -1.7977
    joint_target[1] = -1.3963
    joint_target[2] = 2.601
    joint_target[3] = -1.204
    joint_target[4] = 0.1222
    joint_target[5] = -1.5708
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_12 = np.array([[ 0.162645882, -0.0600150443, 0.984857962, 0.157209], 
                            [-0.345313737, -0.938487311, -0.000162075272, -0.169422], 
                            [0.924286446, -0.340058521, -0.173364534, 0.194884], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_12, matrix)
    
    list[11].aruco_id = A_id
    list[11].position_x = aruco[0,3]
    list[11].position_y = aruco[1,3]
    list[11].position_z = aruco[2,3]

    print("ID:" + str(list[11].aruco_id), "X:" + str(list[11].position_x), "Y:" + str(list[11].position_y), "Z:" + str(list[11].position_z))
    
    # red cube
    waypoints=[]

    wpose.position.x += 0.006914
    wpose.position.y -= 0.002517
    wpose.position.z += 0.133747
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = -1.7453
    joint_target[1] = -1.658
    joint_target[2] = 2.007
    joint_target[3] = 0.2792
    joint_target[4] = 0.2269
    joint_target[5] = -2.1293
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_13 = np.array([[ 0.26418577, -0.16171709, 0.9508159, 0.17328], 
                            [-0.34981767, -0.93477768, -0.0617913, -0.181066], 
                            [ 0.89879413, -0.31628808, -0.30352848, 0.327991], 
                            [ 0, 0, 0, 1]])

    aruco = np.matmul(waypoint_13, matrix)
    
    list[12].aruco_id = A_id
    list[12].position_x = aruco[0,3]
    list[12].position_y = aruco[1,3]
    list[12].position_z = aruco[2,3]

    print("ID:" + str(list[12].aruco_id), "X:" + str(list[12].position_x), "Y:" + str(list[12].position_y), "Z:" + str(list[12].position_z))

    # inspection window place location
    waypoints=[]

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = -1.0996
    joint_target[1] = -1.71
    joint_target[2] = 1.9722
    joint_target[3] = -0.105
    joint_target[4] = 2.112
    joint_target[5] = -1.5
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    wpose.position.x += 0.004823
    wpose.position.y -= 0.007164
    wpose.position.z += 0.04529
    waypoints.append(deepcopy(wpose))

    (plan, fraction) = manipulator_group.compute_cartesian_path (waypoints, 0.01,0.0,True) 
    manipulator_group.execute(plan)

    joint_target = manipulator_group.get_current_joint_values()
    joint_target[0] = -1.2566
    joint_target[1] = -1.2042
    joint_target[2] = 1.222
    joint_target[3] = 1.4486
    joint_target[4] = 1.6232
    joint_target[5] = -1.18682
    manipulator_group.set_joint_value_target(joint_target)

    plan = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix , A_id  = transformation(sub)

    waypoint_14 = np.array([[-0.0717537, -0.99626337, -0.04806783, 0.164638], 
                            [-0.99739707,  0.07132563, 0.01056036, -0.265805], 
                            [-0.00709242, 0.04870046, -0.99878825, 0.299542], 
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_14, matrix)
    
    list[13].aruco_id = A_id
    list[13].position_x = aruco[0,3]
    list[13].position_y = aruco[1,3]
    list[13].position_z = aruco[2,3]

    print("ID:" + str(list[13].aruco_id), "X:" + str(list[13].position_x), "Y:" + str(list[13].position_y), "Z:" + str(list[13].position_z))

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

    # waypoints=[]
    start_pose = manipulator_group.get_current_pose().pose
    wpose = deepcopy(start_pose)
    
    moveit_commander.roscpp_shutdown()

def transformation(msg):
    trans_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0]])

    tag_id = 0

    for m in msg.transforms:
        tag_id = m.fiducial_id
        trans = m.transform.translation
        rot = m.transform.rotation
        t = Transform()

        t.translation.x = trans.x
        t.translation.y = trans.y
        t.translation.z = trans.z
        t.rotation.x = rot.x
        t.rotation.y = rot.y
        t.rotation.z = rot.z
        t.rotation.w = rot.w

        q0 = t.rotation.w
        q1 = t.rotation.x 
        q2 = t.rotation.y
        q3 = t.rotation.z

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 + q0 * q3)
        r02 = 2 * (q1 * q3 - q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 - q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 + q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 + q0 * q2)
        r21 = 2 * (q2 * q3 - q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        trans_matrix = np.array([[r00, r01, r02, t.translation.x],
                        [r10, r11, r12, t.translation.y],
                        [r20, r21, r22, t.translation.z],
                        [0, 0, 0, 1]])

    return trans_matrix, tag_id

def swap(list,size):
    i=0 
    j=0
    for i in range(size):
        for j in range(0,size-i-1):
            if list[j].aruco_id>list[j+1].aruco_id:
                list[j],list[j+1]=list[j+1],list[j]
    return list        

if __name__ == "__main__":
    rospy.init_node('objective1', anonymous=True)
    main()
    list=swap(list,15)
    tagid=[]
    posx=[]
    posy=[]
    posz=[]
    i=0
    for i in range (15):
        tagid.append(list[i].aruco_id)
        posx.append(list[i].position_x)
        posy.append(list[i].position_y)
        posz.append(list[i].position_z)
    dict={'tag_id':tagid,'position_x':posx,'position_y':posy,'position_z':posz}
    df=pd.DataFrame(dict)
    df.to_csv('markers.csv',index=False)
    df.to_csv('~/catkin_ws/src/ERC_2021_simulator/simulation/scripts/markers.csv',index=False)
    print (df)





