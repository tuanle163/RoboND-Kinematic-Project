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
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')		# link offset
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')		# join angles
    alp0, alp1, alp2, alp3, alp4, alp5, alp6 = symbols('alp0:7')	# twist angles

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

	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

	# Extract rotation matrices from the transformation matrices
	#
	#
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
    Rot_EE = Rot_z * Rot_y * Rot_x
    Rot_EE = Rot_EE * Rot_Error

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

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

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
