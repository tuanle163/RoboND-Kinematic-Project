#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') 		# link lengths
	alp0, alp1, alp2, alp3, alp4, alp5, alp6 = symbols('alp0:7')	# twist angles
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')		# link offset
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')		# join angles

	# Create Modified DH parameters
	DH_table = {alp0: 0,		a0: 0,		d1: 0.75,	q1: q1,
		    alp1: -pi/2,	a1: 0.35,	d2: 0,		q2: (-pi/2)+q2,
		    alp2: 0,		a2: 1.25,	d3: 0,		q3: q3,
		    alp3: -pi/2,	a3: -0.054,	d4: 1.5,	q4: q4,
		    alp4: pi/2,		a4: 0,		d5: 0,		q5: q5,
		    alp5: -pi/2,	a5: 0,		d6: 0,		q6: q6,
		    alp6: 0,		a6: 0,		d7: 0.303,	q7: 0}

	# Define Modified DH Transformation matrix
    def TF_matrix(alp, a, d, q):
        TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                        [ sin(q)*cos(alp), cos(q)*cos(alp), -sin(alp), -sin(alp)*d],
                        [ sin(q)*sin(alp), cos(q)*sin(alp),  cos(alp),  cos(alp)*d],
                        [                 0,                 0,           0,             1]])
        return TF

	# Create individual transformation matrices

    T0_1 = TF_matrix(alp0, a0, d1, q1).subs(DH_table)
    T1_2 = TF_matrix(alp1, a1, d2, q2).subs(DH_table)
    T2_3 = TF_matrix(alp2, a2, d3, q3).subs(DH_table)
    T3_4 = TF_matrix(alp3, a3, d4, q4).subs(DH_table)
    T4_5 = TF_matrix(alp4, a4, d5, q5).subs(DH_table)
    T5_6 = TF_matrix(alp5, a5, d6, q6).subs(DH_table)
    T6_EE = TF_matrix(alp6, a6, d7, q7).subs(DH_table)

    T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)

    # Compensate for rotation discrepancy between DH parameters and Gazebo
    # Find EE rotation matrix
    # Roll, Pitch, Yaw
    r, p, y = symbols('r p y')
    # Roll
    Rot_x = Matrix([[1, 0, 0], [0, cos(r), -sin(r)], [0, sin(r), cos(r)]])
    # Pitch
    Rot_y = Matrix([[cos(p), 0, sin(p)], [0, 1, 0], [-sin(p), 0, cos(p)]])
    # Yaw
    Rot_z = Matrix([[cos(y), -sin(y), 0], [sin(y), cos(y), 0], [0, 0, 1]])

    # Fixing rotation error of Kuka KR210 arm
    Rot_Error = Rot_z.subs(y, radians(180)) * Rot_y.subs(p, radians(-90))
    #print("Rotation Error:", Rot_Error)

    Rot_EE = Rot_z * Rot_y * Rot_x
    #print("Rotation EE before sub:", Rot_EE)

    Rot_EE = Rot_EE * Rot_Error

    # Extract rotation matrices from the transformation matrices
	#Rot0_1 = T0_1.col_del(3).row_del(3)
	#Rot1_2 = T1_2.col_del(3).row_del(3)
	#Rot2_3 = T2_3.col_del(3).row_del(3)
	#Rot3_4 = T3_4.col_del(3).row_del(3)
	#Rot4_5 = T4_5.col_del(3).row_del(3)
	#Rot5_6 = T5_6.col_del(3).row_del(3)
	#Rot0_EE = T0_EE.col_del(3).row_del(3)

    ###
    # Initialize service response
    joint_trajectory_list = []
    for x in xrange(0, len(req.poses)):
        # IK code starts here
        joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z
        # Store End-effector position
        EE = Matrix([[px], [py], [pz]])

        # Extract end-effector orientation from quaternion
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x,
            req.poses[x].orientation.y,
            req.poses[x].orientation.z,
            req.poses[x].orientation.w])


        # Assign symbol of roll, pitch, yaw angle
        Rot_EE = simplify(Rot_EE.subs({'r': roll, 'p': pitch, 'y': yaw}))
        #print("Rotation EE after sub:", Rot_EE)

        ### Your IK code here
        WC = EE - (0.303) * Rot_EE[:,2]

	    # Calculate joint angles using Geometric IK method
	    # Calculate theta1 = arctan(WCy, WCx)
        theta1 = atan2(WC[1],WC[0])

	    # Calculate theta2 and theta3
        side_a = 1.501
        side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
        side_c = 1.25

        angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
        angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
        angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

        theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[1] * WC[1] + WC[0] * WC[0]) - 0.35)
        theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sg in link4 of -0.054m

        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

        R3_6 = R0_3.inv("LU") * Rot_EE

        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])
        theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])

	    ###
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

	    # Apply Forward Kinematic to find end-effector position and orientation
        #FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})


    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
