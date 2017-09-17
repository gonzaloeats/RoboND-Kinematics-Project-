## Project: Kinematics Pick & Place
### Writeup : 
---


<!--- # **Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 
-->

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here I drew a sketch of the Kuka arm based kr210.urdf.xacro file displayed in Rviz : 

![alt text][image1]


DH Parameter table:

i | alpha(i-1) | a (i-1) | d (i) | q (i)
--- | --- | --- | --- | ---
1 | 0| 0 | 0.75| 
2 | -pi/2| 0.35| 0| q2: q2-pi/2
3 | 0| 1.25| 0|
4 | -pi/2| -0.054| 1.50| 
5 | pi/2| 0|  0|
6 | -pi/2| 0| 0|
7 |  0| 0| 0.303| q7: 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition,: also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

``` python
 # Modified DH params
        DH_Table = {alpha0:     0, a0:      0, d1:  0.75,
                 alpha1: -pi/2, a1:   0.35, d2:     0, q2: -pi/2.+q2,
                 alpha2:     0, a2:   1.25, d3:     0,
                 alpha3: -pi/2, a3: -0.054, d4:  1.50,
                 alpha4:  pi/2, a4:      0, d5:     0,
                 alpha5: -pi/2, a5:      0, d6:     0,
                 alpha6:     0, a6:      0, d7: 0.303, q7: 0}
    # Define Modified DH Transformation matrix
        def TF_Matrix(alpha, a, d, q):
            TF =    Matrix([[            cos(q),           -sin(q),           0,             a],
                           [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                           [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                           [                 0,                 0,           0,             1]])
            return TF

        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)        
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)        
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)        
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)        
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)        
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```

Transformation from base_link to gripper_link:
```python        
        T0_EE = T0_1* T1_2* T2_3* T3_4* T4_5* T5_6* T6_EE
```



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

First we use `req.poses[]` to get float values for the end effector x,y,and z position.
```python
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
```

Given the position we are able to feed those values back into our transformation matrix to obtain our wrist center.
```python
    EE = Matrix([[px],
                [py],
                [pz]])
    WC = EE - (0.303) * ROT_EE[:,2]
```

Using the WC we can calculate the first three thetas:
```python
 #Calculate joint angles using Geometric IK method
    # More information can be found inthe Iverse Kinematics with Kuka KR210
    theta1 =atan2(WC[1],WC[0]) 



    # SSS triangle for theta2 and theta3
    side_a = 1.501
    side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75),2))
    side_c = 1.25

    angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
    angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))


    theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35) 
    theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sage in link4 for -0.054m
```

Using the rotation matrix we can calculate the remaining Thetas:

```python
  R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    R3_6 = R0_3.inv("LU") * ROT_EE

    theta4 = atan2(R3_6[2,2], -R3_6[0,2]) 
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2]) 
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


This project was a great way to familiarize myself with the ROS environment and the symbolic math library (sympy):

+ Learning the structure of the urdf files started to make sense once I successfully changed the color of my rods. (made my rods green)
+ Managing my .bashrc was a critical learning objective. Especially when I was testing different versions of my code in different directories. (found my self asking where did the shelf and bucket go?!)
+ I found running 16GB of RAM with a dedicated graphics made it much easier to determine if it was my code or my machine that was creating errors and delay.
+ Using the Debug code proved very helpful in understanding the project requirements. 
+ I was successful in grasping the rods and placing them in the bucket most of the times. 

Future improvements:

+ Although I pulled some of the calculations out of the main loop. I would like to store the values as a pickle file and improve the IK calculation speed.


Here is a picture of one of the successful runs:
![alt text][image2]


Thank you to the Slack community for all the help and the updated version of the project with the debug code. Could not have completed this project without all the support.