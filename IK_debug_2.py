import numpy
import math
import tf
from sympy import *
from time import time
from mpmath import radians

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##

    ## Insert IK code here!
    # Your FK code here
    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    R, P, Y = symbols('R P Y')

    # Create Modified DH parameters
    s = {alpha0:      0,  a0:        0, d1:   0.75,
         alpha1:  -pi/2.,  a1:     0.35, d2:      0, q2:  q2-pi/2.,
         alpha2:      0,  a2:     1.25, d3:      0,
         alpha3:  -pi/2.,  a3:   -0.054, d4:   1.50,
         alpha4:   pi/2.,  a4:        0, d5:      0,
         alpha5:  -pi/2.,  a5:        0, d6:      0,
         alpha6:      0,  a6:        0, d7:  0.303, q7:           0}

    # *** Homogeneous Transforms ***
    def Transform_Matrix(alpha, a, d, q):
        TF = Matrix([[             cos(q),            -sin(q),            0,              a],
                     [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                     [                 0,                 0,           0,             1]])
        return TF

    # base link to link 1
    T0_1 = Transform_Matrix(alpha0, a0, d1, q1).subs(s)
    # link 1 to link 2
    T1_2 = Transform_Matrix(alpha1, a1, d2, q2).subs(s)
    # link 2 to link 3
    T2_3 = Transform_Matrix(alpha2, a2, d3, q3).subs(s)
    # link 3 to link 4
    T3_4 = Transform_Matrix(alpha3, a3, d4, q4).subs(s)
    # link 4 to link 5
    T4_5 = Transform_Matrix(alpha4, a4, d5, q5).subs(s)
    # link 5 to link 6
    T5_6 = Transform_Matrix(alpha5, a5, d6, q6).subs(s)
    # link 6 to gripper
    T6_G = Transform_Matrix(alpha6, a6, d7, q7).subs(s)


    # DH to URDF to rotation
    #R_corr = Matrix([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])

    # Total homogeneous transform
    # link 0 to gripper
    # T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
    T0_G = Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)], [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)], [-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

    T_total = Matrix([[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)], [-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)], [-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3), -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

    ###

    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    R_x = Matrix([[ 1,       0,       0],
                  [ 0,  cos(R), -sin(R)],
                  [ 0,  sin(R),  cos(R)]])

    R_y = Matrix([[ cos(P),    0,  sin(P)],
                  [      0,    1,       0],
                  [-sin(P),    0,  cos(P)]])

    R_z = Matrix([[ cos(Y),  -sin(Y),        0],
                  [ sin(Y),   cos(Y),        0],
                  [      0,        0,        1]])

    Rrpy = R_z * R_y * R_x
    #print(Rrpy)
    R_corr = R_z.subs(Y, pi) * R_y.subs(P, -pi/2.)
    #print(R_corr)

    # DH to URDF to rotation
    R_corr = Matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    Rrpy = Matrix([[cos(P)*cos(Y), sin(P)*sin(R)*cos(Y) - sin(Y)*cos(R), sin(P)*cos(R)*cos(Y) + sin(R)*sin(Y)], [sin(Y)*cos(P), sin(P)*sin(R)*sin(Y) + cos(R)*cos(Y), sin(P)*sin(Y)*cos(R) - sin(R)*cos(Y)], [-sin(P), sin(R)*cos(P), cos(P)*cos(R)]])


    # Compensate for rotation discrepancy between DH parameters and Gazebo
    Rrpy = Rrpy * R_corr
    Rrpy = Rrpy.subs({R: roll, P: pitch, Y: yaw})
    #Rrpy = Rrpy[:-1, :-1]

    r_EE = Matrix([[px],
                  [py],
                  [pz]])

    r_WC = (r_EE - d7 * Rrpy[:, 2]).subs(s)
    #
    #
    # Calculate joint angles using Geometric IK method
    #
    #
    # calculate theta1 from WC rotating around XY plane
    theta1 = atan2(r_WC[1], r_WC[0])

    # calculate scalene triangle between R2,R3, and WC (R5)
    A = d4.subs(s)
    C = a2.subs(s)

    # B is calculated using a right triangle drawn about WC.
    # First term projects XY WC components onto x-axis and subtracts distance from base to link_3 (a1)
    proj_x = (sqrt(r_WC[0]**2 + r_WC[1]**2) - a1).subs(s)
    # print("proj_x: ", proj_x)
    # second term takes WC[2] (z) and subtracts distance from base to link_3 d1
    proj_z = (r_WC[2] - d1).subs(s)
    # print("proj_z: ", proj_z)
    # pythagorous
    B = sqrt((proj_x)**2 + (proj_z)**2)
    # angles from law of cosines
    angle_A = acos((B**2+C**2-A**2)/(2*B*C))
    angle_B = acos((A**2+C**2-B**2)/(2*A*C))
    angle_C = acos((A**2+B**2-C**2)/(2*A*B))

    # calculate theta2 and theta3 using exterior angle analysis
    theta2 = (pi/2 - angle_A - atan2(proj_z, proj_x))
    # this gives a large error for some reason....
    # theta3 = (pi/2. - (angle_B - atan2(a3, d4)).subs(s))
    theta3 = pi/2 - (angle_B + 0.036)

    # create transform matrix R0_3
    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

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

    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    # T_total, T0_G
    FK = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [r_WC[0],r_WC[1],r_WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
