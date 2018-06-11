#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Original author: Harsh Pandya
# Modified by Ray Tang 6/10/2018

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from mpmath import radians


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #7 is the gripper
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	
	# Create Modified DH parameters
	a10 = 0.35 #from URDF file
	a20 = 1.25
	a30 = -0.054
	d10 = 0.75
	d40 = 1.5
	d70 = 0.303
	
	s = {alpha0: 	  0,  a0:   0,  d1: d10,  q1:       q1,
	     alpha1:  -pi/2.,  a1: a10,  d2:   0,  q2: q2-pi/2.,
	     alpha2: 	  0,  a2: a20,  d3:   0,  q3:       q3,
     	     alpha3: -pi/2.,  a3: a30,  d4: d40,  q4:       q4,	
	     alpha4:  pi/2.,  a4:   0,  d5:   0,  q5:       q5,
	     alpha5: -pi/2.,  a5:   0,  d6:   0,  q6:       q6, 
	     alpha6:      0,  a6:   0,  d7:   d70,q7:        0}

	
	# Define Modified DH Transformation matrix
	def DH(alpha,a,d,q):
	    DH = Matrix([[             cos(q),            -sin(q),            0,              a],
        	       [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        	       [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        	       [                   0,                   0,            0,               1]])
	    return DH

	# Create individual transformation matrices
	T0_1 = DH(alpha0,a0,d1,q1).subs(s)
	T1_2 = DH(alpha1,a1,d2,q2).subs(s)
	T2_3 = DH(alpha2,a2,d3,q3).subs(s)
	T3_4 = DH(alpha3,a3,d4,q4).subs(s)
	T4_5 = DH(alpha4,a4,d5,q5).subs(s)
	T5_6 = DH(alpha5,a5,d6,q6).subs(s)
	T6_7 = DH(alpha6,a6,d7,q7).subs(s)

	T0_7 = T0_1*T1_2*T2_3 * T3_4 * T4_5 * T5_6* T6_7 #base link to gripper

	#
	# Extract rotation matrices from the transformation matrices
	#
	#
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
	    
	    Gripper = Matrix([[px],
			      [py],
			      [pz]]); #matrix for the gripper position 
	    
	    # roation matrix	
	    r, p, y = symbols('r p y')
	    
	    R_x = Matrix([[1, 0, 0],
        	          [0, cos(r), -sin(r)],
        	          [0, sin(r), cos(r)]])
	    
	    R_y = Matrix([[cos(p), 0, sin(p)],
        	          [0, 1, 0],
        	          [-sin(p),0, cos(p)]])

	    R_z = Matrix([[cos(y), -sin(y), 0],
        	          [sin(y), cos(y), 0],
        	          [0, 0, 1]])
	
	    R_rpy = R_z*R_y*R_x	#rotation matrix for the end effector


            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    
	    Rot_corr = R_z.subs(y, radians(180))*R_y.subs(p, radians(-90)) #corrections for the discrepancies between DH and gazebo
	    
	    R_rpy = R_rpy*Rot_corr #per section 15 
	    
	    R_rpy = R_rpy.subs({'r':roll, 'p':pitch, 'y':yaw})

	    WC = Gripper - d70*R_rpy[:,2]  #WC is the wrist center position
		
	    # Calculate joint angles using Geometric IK method
	    
	    theta1 = atan2(WC[1],WC[0])	    #theta 1 

	    # solve theta 2 and theta 3 using notations in section 15
	    # first calculate length of three sides:
	    sC = 1.25
	    sA = 1.501 # sqrt(d40^2 + a30^2)
	    sB = sqrt(pow((WC[2]-0.75),2)+pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2)) #distance between WC at joint 5 and joint 2 	
	    # now calculate theta a, b, c of the triangle, given sC, sB, and sA (cosin law)
	    thetaa = acos((sB*sB + sC*sC - sA*sA)/(2*sB*sC))
	    thetab = acos((sA*sA + sC*sC - sB*sB)/(2*sA*sC))
	    thetac = acos((sB*sB + sA*sA - sC*sC)/(2*sA*sB))	
	    
	    # now calculate theta 2 and theta 3
	    theta2 = pi/2 - thetaa - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)	
	    theta3 = (pi/2-0.036) - thetab #1.5348 is the angle at the original position

	    # now calculate 4, 5, and 6
	    R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
	    R0_3 = R0_3.evalf(subs = {q1:theta1, q2:theta2, q3:theta3})
	    R3_6 = R0_3.inv("LU")*R_rpy

	    #https://pdfs.semanticscholar.org/6681/37fa4b875d890f446e689eea1e334bcf6bf6.pdf
	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
		
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
