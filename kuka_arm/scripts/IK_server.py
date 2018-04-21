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
from sympy import Matrix, cos, sin, atan2, sqrt, acos, symbols, simplify
from numpy import pi, float64, linspace


# global variables for symbolic representation
# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
# for orientation
R, P, Y = symbols('R P Y')


# *** Homogeneous Transforms ***
def Transform_Matrix(alpha, a, d, q):
    TF = Matrix([[cos(q),            -sin(q),            0,              a],
                 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                 [0,                 0,           0,             1]])
    return TF


def rot_x(q):
    R_x = Matrix([[1,              0,        0],
                  [0,         cos(q),  -sin(q)],
                  [0,         sin(q),  cos(q)]])

    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q),        0,  sin(q)],
                  [0,        1,       0],
                  [-sin(q),        0, cos(q)]])

    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q),  -sin(q),       0],
                  [sin(q),   cos(q),       0],
                  [0,        0,       1]])
    return R_z


def rotation_matrix(rz, ry, rx):
    # define symbols of orientation of EE
    oR, oP, oY = symbols('oR oP oY')
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    # DH to URDF to rotation
    Rot_EE = simplify(rot_z(Y) * rot_y(P) * rot_x(R))
    R_corr = rot_z(rz) * rot_y(ry) # * rot_x(rx)
    Rot_EE = Rot_EE * R_corr

    return(Rot_EE)

# remap joints in path along a linear trajectory from the start to end joint angle
def remap(joint_number, joint_list):
    new_joints = [row[joint_number] for row in joint_list]
    list_length = len(new_joints) - 3
    mapped = linspace(new_joints[0], new_joints[list_length - 1], num=list_length)
    for i in range(list_length):
        joint_list[i][joint_number] = mapped[i]

    return joint_list

# ********** IK SOLVER ***********
# ********************************


# debugging became an issue. Decided to turn into an object for easier debugging.
# inspired from ref: https://github.com/NitishPuri/RoboND-Kinematics-Project
class KukaIKSolver(object):
    def __init__(self):

        # Create Modified DH parameters
        self.DH = {'alpha0':       0,  'a0':        0, 'd1':   0.75,
                   'alpha1': -pi / 2,  'a1':     0.35, 'd2':      0, q2: q2 - pi / 2,
                   'alpha2':       0,  'a2':     1.25, 'd3':      0,
                   'alpha3': -pi / 2,  'a3':   -0.054, 'd4':   1.50,
                   'alpha4':  pi / 2,  'a4':        0, 'd5':      0,
                   'alpha5': -pi / 2,  'a5':        0, 'd6':      0,
                   'alpha6':       0,  'a6':        0, 'd7':  0.303, q7: 0}

        # base link to link 1
        T0_1 = Transform_Matrix(self.DH['alpha0'], self.DH['a0'], self.DH['d1'], q1).subs(self.DH)
        # link 1 to link 2
        T1_2 = Transform_Matrix(self.DH['alpha1'], self.DH['a1'], self.DH['d2'], q2).subs(self.DH)
        # link 2 to link 3
        T2_3 = Transform_Matrix(self.DH['alpha2'], self.DH['a2'], self.DH['d3'], q3).subs(self.DH)
        # link 3 to link 4
        ##T3_4 = Transform_Matrix(self.DH['alpha3'], self.DH['a3'], self.DH['d4'], q4).subs(self.DH)
        # link 4 to link 5
        ##T4_5 = Transform_Matrix(self.DH['alpha4'], self.DH['a4'], self.DH['d5'], q5).subs(self.DH)
        # link 5 to link 6
        ##T5_6 = Transform_Matrix(self.DH['alpha5'], self.DH['a5'], self.DH['d6'], q6).subs(self.DH)
        # link 5 to link 6
        ##T6_G = Transform_Matrix(self.DH['alpha6'], self.DH['a6'], self.DH['d7'], q7).subs(self.DH)

        # composition of homogeneous transform
        #self.T0_2 = simplify(T0_1 * T1_2)
        #self.T0_3 = simplify(self.T0_2 * T2_3)
        #self.T0_4 = simplify(self.T0_3 * T3_4)
        #self.T0_5 = simplify(self.T0_4 * T4_5)
        #self.T0_6 = simplify(self.T0_5 * T5_6)
        #self.T0_G = simplify(self.T0_6 * T6_G)

        #T0_2 = simplify(T0_1 * T1_2)
        #T0_3 = simplify(T0_2 * T2_3)

        # create rotation matrix due to discrepancy between DH to URDF
        # coord systems
        self.R_corr = rotation_matrix(pi, -pi / 2, 0)

        # create transform matrix R0_3
        self.R0_3 = simplify(T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3])

        # joints
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.theta5 = 0
        self.theta6 = 0

        # track previous wrist joint positions
        self.oldTheta = []
        # temporary solution to solving path fluidity by remapping J4-6
        # to a linear space along the path
        self.remap = False

    def solveJ1_J3(self, r_WC):
        # calculate theta1 from WC rotating around XY plane
        self.theta1 = float64(atan2(r_WC[1], r_WC[0]))

        # calculate scalene triangle between R2,R3, and WC (R5)
        # calculated in rViz
        A = 1.501

        ##C = self.DH['a2']
        C = 1.25

        # B is calculated using a right triangle drawn about WC.
        # First term projects XY WC components onto x-axis and subtracts distance from base to link_3 (a1)
        proj_x = (sqrt(r_WC[0]**2 + r_WC[1]**2) - self.DH['a1'])
        # second term takes WC[2] (z) and subtracts distance from base to link_3 d1
        proj_z = (r_WC[2] - self.DH['d1'])
        # Use pythagorous theorem to calculate B
        B = sqrt((proj_x)**2 + (proj_z)**2)
        # angles from law of cosines
        angle_A = acos((B**2 + C**2 - A**2) / (2 * B * C))
        angle_B = acos((A**2 + C**2 - B**2) / (2 * A * C))
        # angle_C = acos((A**2+B**2-C**2)/(2*A*B))

        # calculate theta2 and theta3 using exterior angle analysis
        self.theta2 = float64(pi / 2 - angle_A - atan2(proj_z, proj_x))
        # using asin function gave incorrect result for some reason, had to
        # precompute asin(a3/d4)
        self.theta3 = float64(pi / 2 - (angle_B + 0.036))

    def solveJ3_J6(self, R):
        # Find theta4 theta5 theta6 using Euler angles from rotation matrix
        # Identify useful terms from rotation matrix
        r12 = R[0, 1]
        r13 = R[0, 2]
        r21 = R[1, 0]
        r22 = R[1, 1]
        r23 = R[1, 2]
        r32 = R[2, 1]
        r33 = R[2, 2]

        # Euler Angles from Rotation Matrix
        # sympy synatx for atan2 is atan2(y, x)
        if abs(r23) != 1:
            self.theta5 = float64(atan2(sqrt(r13**2 + r33**2), r23))
            if (sin(self.theta5) < 0):
                self.theta4 = float64(atan2(-r33, r13))
                self.theta6 = float64(atan2(r22, -r21))
            else:
                self.theta4 = float64(atan2(r33, -r13))
                self.theta6 = float64(atan2(-r22, r21))
        else:
            self.theta6 = self.old_theta6
            if r23 == 1:
                self.theta5 = float64(0)
                self.theta4 = float64(atan2(-r12, -r32) - self.theta6)
            else:
                self.theta5 = float64(0)
                self.theta4 = float64(self.theta6 - atan2(r12, -r32))

# -------------- IK_Server specific --------------------------

    def handle_calculate_IK(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print("No valid poses received")
            return -1
        else:

            # Initialize service response
            joint_trajectory_list = []
            for x in xrange(0, len(req.poses)):
                # IK code starts here
                joint_trajectory_point = JointTrajectoryPoint()

                # show progress
                print('Calculating %s from %s' % (x + 1, len(req.poses)))

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

                # calculate rotation matrix
                R_corr = self.R_corr.subs({R: roll, P: pitch, Y: yaw})

                # end effector position
                r_EE = Matrix([[px],
                               [py],
                               [pz]])

                # calcualte wrist center position
                ##r_WC = (r_EE - self.DH['d7'] * R_corr[:, 2])
                r_WC = (r_EE - (0.303) * R_corr[:, 2])

                # calculate theta1-3
                self.solveJ1_J3(r_WC)

                # create rotation matrix for last three joints R3_6
                # create rotation matrix for last three joints R3_6
                R0_3 = self.R0_3.evalf(subs={q1: self.theta1, q2: self.theta2, q3: self.theta3})
                R3_6 = R0_3.inv("LU") * R_corr

                # calculate theta4-6
                self.solveJ3_J6(R3_6)

                # save old joint values
                self.oldTheta.append([self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6])

                if not self.remap:
                    # Populate response for the IK request
                    # In the next line replace theta1,theta2...,theta6 by your joint angle variables
                    joint_trajectory_point.positions = [self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6]
                    joint_trajectory_list.append(joint_trajectory_point)

            ### Temp solution to J4-6 problem
            # remap wrist joints between path for fluid motions
            if self.remap:
                joint_list = self.oldTheta
                # j4
                joint_list = remap(3, joint_list)
                # j5
                joint_list = remap(4, joint_list)
                # j6
                joint_list = remap(5, joint_list)
                for i in range(len(joint_list)):
                    # Populate response for the IK request
                    # In the next line replace theta1,theta2...,theta6 by your joint angle variables
                    joint_trajectory_point.positions = joint_list[i]
                    joint_trajectory_list.append(joint_trajectory_point)
            ########

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)

# ******* end KukaIKSolver


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    # initialize solver
    robot = KukaIKSolver()
    s = rospy.Service('calculate_ik', CalculateIK, robot.handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
