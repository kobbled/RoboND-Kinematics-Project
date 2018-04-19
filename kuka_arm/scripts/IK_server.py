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
from sympy import Matrix, cos, sin, atan2, pi, sqrt, acos

# *** Homogeneous Transforms ***
def Transform_Matrix(alpha, a, d, q):
    TF = Matrix([[             cos(q),            -sin(q),            0,              a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return TF

def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(q),  -sin(q)],
                  [ 0,         sin(q),  cos(q)]])

    return R_x

def rot_y(q):
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [      0,        1,       0],
                  [-sin(q),        0, cos(q)]])

    return R_y

def rot_z(q):
    R_z = Matrix([[ cos(q),  -sin(q),       0],
                  [ sin(q),   cos(q),       0],
                  [      0,        0,       1]])
    return R_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:

        ## Insert IK code here!
        # Your FK code here
        # Create symbols
        #q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Create Modified DH parameters
        DH = {'alpha0':      0,  'a0':        0, 'd1':   0.75,
             'alpha1':  -pi/2,  'a1':     0.35, 'd2':      0,
             'alpha2':      0,  'a2':     1.25, 'd3':      0,
             'alpha3':  -pi/2,  'a3':   -0.054, 'd4':   1.50,
             'alpha4':   pi/2,  'a4':        0, 'd5':      0,
             'alpha5':  -pi/2,  'a5':        0, 'd6':      0,
             'alpha6':      0,  'a6':        0, 'd7':  0.303}

        # base link to link 1
        ##T0_1 = Transform_Matrix(alpha0, a0, d1, q1).subs(DH)
        ##T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0.750000000000000], [0, 0, 0, 1]])
        # link 1 to link 2
        ##T1_2 = Transform_Matrix(alpha1, a1, d2, q2).subs(DH)
        ##T1_2 = Matrix([[sin(q2), cos(q2), 0, 0.350000000000000], [0, 0, 1, 0], [cos(q2), -sin(q2), 0, 0], [0, 0, 0, 1]])
        # link 2 to link 3
        ##T2_3 = Transform_Matrix(alpha2, a2, d3, q3).subs(DH)
        ##T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25000000000000], [sin(q3), cos(q3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


        # Total homogeneous transform
        # link 0 to gripper
        # T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
        ###

        # define symbols of orientation of EE
        #R, P, Y = symbols('R P Y')
        # apply rotation matricies
        ## Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)


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

            # Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            # DH to URDF to rotation
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
            R_corr = rot_z(pi) * rot_y(-pi/2)
            Rrpy = Rrpy * R_corr

            # sub in values retrieved from request
            #Rrpy = Rrpy.subs({R: roll, P: pitch, Y: yaw})

            r_EE = Matrix([[px],
                          [py],
                          [pz]])

            r_WC = (r_EE - DH['d7'] * Rrpy[:, 2])
            #
            #
            # Calculate joint angles using Geometric IK method
            #
            #
            # calculate theta1 from WC rotating around XY plane
            theta1 = atan2(r_WC[1], r_WC[0])

            # calculate scalene triangle between R2,R3, and WC (R5)
            A = DH['d4']
            C = DH['a2']

            # B is calculated using a right triangle drawn about WC.
            # First term projects XY WC components onto x-axis and subtracts distance from base to link_3 (a1)
            proj_x = (sqrt(r_WC[0]**2 + r_WC[1]**2) - DH['a1'])
            # second term takes WC[2] (z) and subtracts distance from base to link_3 d1
            proj_z = (r_WC[2] - DH['d1'])
            #Use pythagorous theorem to calculate B
            B = sqrt((proj_x)**2 + (proj_z)**2)
            # angles from law of cosines
            angle_A = acos((B**2+C**2-A**2)/(2*B*C))
            angle_B = acos((A**2+C**2-B**2)/(2*A*C))
            angle_C = acos((A**2+B**2-C**2)/(2*A*B))

            # calculate theta2 and theta3 using exterior angle analysis
            theta2 = (pi/2 - angle_A - atan2(proj_z, proj_x))
            # using asin function gave incorrect result for some reason, had to
            # precompute asin(a3/d4)
            theta3 = pi/2 - (angle_B + 0.036)

            # create transform matrix R0_3
            ##R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            ##R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R0_3 = Matrix([[sin(theta2)*cos(theta1)*cos(theta3) + sin(theta3)*cos(theta1)*cos(theta2), -sin(theta2)*sin(theta3)*cos(theta1) + cos(theta1)*cos(theta2)*cos(theta3), -sin(theta1)],
                           [sin(theta1)*sin(theta2)*cos(theta3) + sin(theta1)*sin(theta3)*cos(theta2), -sin(theta1)*sin(theta2)*sin(theta3) + sin(theta1)*cos(theta2)*cos(theta3), cos(theta1)],
                           [-sin(theta2)*sin(theta3) + cos(theta2)*cos(theta3), -sin(theta2)*cos(theta3) - sin(theta3)*cos(theta2), 0]])

            R3_6 = R0_3.inv("LU") * Rrpy

            # Find theta4 theta5 theta6 using Euler angles from rotation matrix
            # Identify useful terms from rotation matrix
            r21 = R3_6[1, 0]
            r22 = R3_6[1, 1]
            r13 = R3_6[0, 2]
            r23 = R3_6[1, 2]
            r33 = R3_6[2, 2]

            # Euler Angles from Rotation Matrix
            # sympy synatx for atan2 is atan2(y, x)
            theta4 = atan2(r33, -r13)
            theta5 = atan2(sqrt(r13*r13 + r33*r33), r23)
            theta6 = atan2(-r22, r21)

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
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
