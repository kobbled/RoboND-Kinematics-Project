## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/2018-04-18_13-37-13.png
[DHParam]: ./misc_images/DH_link_assignment.png
[generalTrans]: ./misc_images/dh-transform-matrix.png
[rotMatrix]: ./misc_images/eq3.png
[frameEq]: ./misc_images/frame_equation.png
[imageIKDecouple]: ./misc_images/joint_decouple.png
[image3]: ./misc_images/misc2.png
[image_WCProblem]: ./misc_images/WC_problem.png
[image_grabPart]: ./misc_images/grabbing_part.png
[image_putInBin]: ./misc_images/inBin.png
[image_success]: ./misc_images/8_outof_10.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The image below shows the kuka KR210 robot and the kinematic chain created in RViz

![alt text][image1]

The DH parameters for the kuka robot are can be visualized as:

![DH visualized][DHParam]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---  | ---    | ---    | ---  | ---
0->1 | 0      | 0      | 0.75 | qi
1->2 | - pi/2 | 0.35   | 0    | -pi/2 + q2
2->3 | 0      | 1.25   | 0    |
3->4 |  -pi/2 | -0.054 | 1.50 |
4->5 |  pi/2  | 0      | 0    |
5->6 | -pi/2  | 0      | 0    |
6->EE| 0      | 0      | 0.303| 0

Transformation matricies for each joint are as follows
``` python
T0_1 = Matrix([
          [cos(q1), -sin(q1), 0,    0],
          [sin(q1),  cos(q1), 0,    0],
          [      0,        0, 1, 0.75],
          [      0,        0, 0,    1]])

T1_2 = Matrix([
          [sin(q2),  cos(q2), 0, 0.35],
          [      0,        0, 1,    0],
          [cos(q2), -sin(q2), 0,    0],
          [      0,        0, 0,    1]])

T2_3 = Matrix([
          [cos(q3), -sin(q3), 0, 1.25],
          [sin(q3),  cos(q3), 0,    0],
          [      0,        0, 1,    0],
          [      0,        0, 0,    1]])

T3_4 = Matrix([
          [ cos(q4), -sin(q4), 0, -0.054],
          [       0,        0, 1,    1.5],
          [-sin(q4), -cos(q4), 0,      0],
          [       0,        0, 0,      1]])

T4_5 = Matrix([
          [cos(q5), -sin(q5),  0, 0],
          [      0,        0, -1, 0],
          [sin(q5),  cos(q5),  0, 0],
          [      0,        0,  0, 1]])
T5_6 = Matrix([
          [ cos(q6), -sin(q6), 0, 0],
          [       0,        0, 1, 0],
          [-sin(q6), -cos(q6), 0, 0],
          [       0,        0, 0, 1]])

T6_G =  Matrix([
          [1, 0, 0,     0],
          [0, 1, 0,     0],
          [0, 0, 1, 0.303],
          [0, 0, 0,     1]])
```

or generalized as:

![general transform equation][generalTrans]

The combined homogeneous transform matrix is:

``` python
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```

where rotational matricies can be decomposed from the transformation matrices as follows:

![rotational decomposition][rotMatrix]

To solve for $\theta_{4} - \theta_{6}$ the equation below can be utilized which relates the end effector position with repect to the wrist center to the base frame.

![reference frame equation][frameEq]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

> theta1 is calculated by solving for the angle of _WC_ w.r.t the XY plane
> \[\theta_{1} = tan^{-1}(\frac{WC_{y}}{WC_{x}})\]

> theta2 and theta3 are solved for geometrically by solving for the angles, and lengths of the triangle using law of cosines:
>\[c = cos^{-1}\frac{(A^{2}+B^{2}+C^{2})}{2AB}\]
>![alt text][imageIKDecouple]
> \[A = d_{4} ,\quad C = a_{2}\]
> \[proj_{x} = \sqrt{wc_{x}^{2} + wc_{y}^{2}} - a_{1}\]
> \[B = \sqrt{proj_{x}^{2} + (wc_z - d_{1})^{2}}\]
> Now that we have _A, B, and C_ we can calculate _a,b,c_ using the law of cosines.

 Therefore, $\theta_{2}$, and $\theta_{3}$ are:
> \[ \theta_{2} = \Big(\pi/2 - a - tan^{-1}\Big(\frac{wc_z - d_{1}}{proj_{x}}\Big)\Big)  \]
> \[ \theta_{3} = \Big( \pi/2 - b - sin^{-1}\Big( \frac{a_{3}}{A} \Big)\Big) \]

To solve for $\theta_{4}$, $\theta_{5}$, $\theta_{6}$ we must know the rotation rotation_matrix between link 3 and link 6, **$R_{3-6}$** :

 R |1                                   | 2                                            | 3
 --- |---                                 | ---                                          | ---
 **1**  |$sin(q6) + cos(q4)*cos(q5)*cos(q6)$ | $-sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5)$ | $-sin(q5)*cos(q4)$
 **2**  |$sin(q5)*cos(q6)$                   | $-sin(q5)*sin(q6)$                           | $cos(q5)$
 **3**  |$-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4)$ | $sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6)$  | $sin(q4)*sin(q5)$*

Using trigonometric identities to solve for $\theta_{4}$, $\theta_{5}$, $\theta_{6}$:

> \[\theta_{4} = tan^{-1}\Big(\frac{R_{33}}{-R_{13}}\Big)\]
> \[\theta_{5} = tan^{-1}\Big(\frac{\sqrt{R_{13}^{2} + R_{33}^{2}}}{R_{23}}\Big)\]
> \[\theta_{6} = tan^{-1}\Big(\frac{R_{22}}{-R_{21}}\Big)\]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

After running an optimized version of the code in IK_server.py in IK_debug.py the following results were output, confirming that $\theta_{1}-\theta_{6}$ are being derived properly with minimal error.

```
Total run time to calculate joint angles from pose is 0.2030 seconds

Wrist error for x position is: 0.00000046
Wrist error for y position is: 0.00000032
Wrist error for z position is: 0.00000545
Overall wrist offset is: 0.00000548 units

Theta 1 error is: 0.00093770
Theta 2 error is: 0.00096560
Theta 3 error is: 0.00312069
Theta 4 error is: 0.00153782
Theta 5 error is: 0.00184772
Theta 6 error is: 0.00225943

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have muliple positions. It is best to add your forward kinmeatics to            
confirm whether your code is working or not**


End effector error for x position is: 0.00076951
End effector error for y position is: 0.00058612
End effector error for z position is: 0.00009502
Overall end effector offset is: 0.00097197 units
```

Running IK_server.py however highlighted some issues with the IK alogrithm utilized. While the robot conformed to the path created by MoveIt, the motion was not fluid, and took a very long time to get to the final position in the path. Joints 4-6 rotated very speratically, moving move than 180 degrees in some cases from one position to the next.

![wrist center problem][image_WCProblem]

A new implementation calculating the wrist center differently could potentially solve this problem. The current alrogithm also does not take into account the previous joint positions, and does not back propogate the trajectory list to see if the wrist center oreintation vector aligns with the calculated path. Correcting joints 4-6 using joint history, or correcting the wrist center orientation through back propagation could help make the robot motion more fluid, and faster.

The robot was successful in picking up and placing the cylinders in the bin most of the time.

![grabbing cylinder][image_grabPart]
![placing cylinder][image_putInBin]

However, the robot did have problems with cylinders placed in the bottom right bin. The image below confirms that the robot was successful 8 out of 10 times.

![pick and place success][image_success]
