## Project: Kinematics Pick & Place
## Ray Tang, 06/10/2018
---
**Goals of the project:**

Goals of the project are to familiarize with the forward and inverse kinematics techniques, code in a python script to control a Kuka KR210 robotic arms, and use it to pick and place objects in a Gazebo simulated environment

**Steps that I took to complete the project:**  

1. Set up the ROS Workspace.
2. Clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

[//]: # (Image References)

[image1]: ./Initial.png
[image2]: ./pick.png
[image3]: ./place.png
[image4]: ./drop.png
[image5]: ./image-4.png

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot 

#### 2. Derive DH parameter table from the URDF file, shown as below

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 3. Create individual transformation matrices at each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose. Mathetically, the eventual homogeneous transform expressed as:

`T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6* T6_G`

where G is short for gripper, T(i)_(i+1) is the transform from link i to (i+1).

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics (for the WC) and inverse Orientation Kinematics (for the end effector). Through the exercise, all three angles (q1 to q6 in th DH table) shall be determined.

The first part is to derive position of the wrist center (WC), which dicates q1, q2 and q3. From the formula shown below

![alt text][image5]


where d = 0.303 is the distance between the gripper and joint 5 (where WC located). We will further need to determine rotation matrix from base link to joint 6 (`Rot0_6`), which can be mathematically determined by:

`Rot0_6 = R_z * R_y * R_z * R_z.subs(y, radians(180))*R_y.subs(p, radians(-90))` and 
`Rot0_6 = Rot0_6.subs({'r':roll, 'p':pitch, 'y':yaw})`
	    
where `R_x`, `R_y`, and `R_z`is respectively rotation matrix around `x`, `y`, and `z` axis. `R_z.subs(y, radians(180))*R_y.subs(p, radians(-90))` are required corrections for reconciling the discrepancies between DH and gazebo conventions.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here is the initial configuration of the Kuka arm. the blue object is positioned at "5".
![alt text][image1]

Here is after the EE reached to the desired position and grabbed the object.
![alt text][image2]

Here is after the EE reached the desired position for placement.
![alt text][image3]

Here is after the EE released the object into the bucket.
![alt text][image4]


