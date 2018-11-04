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

#initializing parameters ,matrices that need to be excuted only once
def init_parameters():
    
    
    #Declaring Joints Parameters & sympols
    global r, p, y   
    global q1, q2, q3, q4, q5, q6, q7 
    global T0_3,T0_7,Rrpy0
    
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    #Initializing Joints Parameters
    #Defining DH Table for kuka arm
    DH_table = {alpha0: 0, a0: 0, d1: 0.75, q1: q1,
                    alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
                    alpha2: 0, a2: 1.25, d3: 0.75, q3: q3,
                    alpha3: -pi / 2, a3: -0.0054, d4: 1.5, q4: q4,
                    alpha4: pi / 2, a4: 0, d5: 0, q5: q5,
                    alpha5: -pi / 2, a5: 0, d6: 0, q6: q6,
                    alpha6: 0, a6: 0, d7: 0.303, q7: 0}
    ###  FK code here
    #Defining Transformation matrices Between Joints
    T0_1 = Transform_Mat(alpha0, a0, d1, q1).subs(DH_table)
    T1_2 = Transform_Mat(alpha1, a1, d2, q2).subs(DH_table)
    T2_3 = Transform_Mat(alpha2, a2, d3, q3).subs(DH_table)
    T3_4 = Transform_Mat(alpha3, a3, d4, q4).subs(DH_table)
    T4_5 = Transform_Mat(alpha4, a4, d5, q5).subs(DH_table)
    T5_6 = Transform_Mat(alpha5, a5, d6, q6).subs(DH_table)
    T6_7 = Transform_Mat(alpha6, a6, d7, q7).subs(DH_table)
    #FK Transformation Matrix from base to End Effector
    T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
    T0_3 = T0_1 * T1_2 * T2_3
    r, p, y = symbols('r p y')
    R_x = rot_x(r)
    R_y = rot_y(p)
    R_z = rot_z(y)
    #correction matrix for end effector
    R_Correct =  rot_z(pi)* rot_y(-pi/2)
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    Rrpy0 = R_z * R_y * R_x * R_Correct
    
 #define transformation matrices bet joints given DH parameters   
def Transform_Mat(alpha, a, d, q):
    TF = Matrix([[cos(q), -sin(q), 0, a],
                 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                 [0, 0, 0, 1]
                 ])
    return TF

### Define functions for homogeneous transform matrices of rotations about x, y, and z given specific angle.
def rot_x(q):
    R_x = Matrix([[1, 0, 0, 0],
                  [0, cos(q), -sin(q), 0],
                  [0, sin(q), cos(q), 0],
                 [0, 0, 0, 1]])

    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q), 0],
                  [0, 1, 0, 0],
                  [-sin(q), 0, cos(q), 0],
                  [0, 0, 0, 1]])

    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0, 0],
                  [sin(q), cos(q), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    return R_z

    
def handle_calculate_IK(req):
    #gobal variables that we will need from init fx
    global q1, q2, q3, q4, q5, q6, q7 
    global T0_3,T0_7,Rrpy0
    global r, p, y 
    
      
    

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
	

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
            T_final= T0_7 

            ### Your IK code here
	    
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            #calculating wrist center
            Rrpy = Rrpy0.subs({'r': roll, 'p': pitch, 'y': yaw})
            print 'Calculating wrist center'
            nx = Rrpy[0, 2]
            ny = Rrpy[1, 2]
            nz = Rrpy[2, 2]

            Wc_x = px - 0.303 * nx
            Wc_y = py - 0.303 * ny
            Wc_z = pz - 0.303 * nz
            #calculate first 3 joints angles
            print 'Calculating position angles'
            C = 1.25  
            A = 1.501
            R_Wc_xzPlane = sqrt(Wc_x ** 2 + Wc_y ** 2)-.35
            B = sqrt(R_Wc_xzPlane ** 2 + (Wc_z - .75) ** 2)
            a_angle = acos((B ** 2 + C ** 2 - A ** 2) / (2 * B * C))
            b_angle = acos((A ** 2 + C ** 2 - B ** 2) / (2 * A * C))
            theta1 = atan2(Wc_y, Wc_x)
            theta2 = pi / 2 - a_angle - atan2(Wc_z - 0.75, R_Wc_xzPlane)
            theta3 = pi / 2 - (b_angle + .036)
            #calculate last 3 joints angles
            print 'Calculating orientation angles'
            
            R0_3 = T0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})
            R3_6 = R0_3.T * Rrpy
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
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
    init_parameters()
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
