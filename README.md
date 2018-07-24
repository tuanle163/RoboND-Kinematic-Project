# <span style="color:teal"> Project: Kinematics Pick & Place

### Writeup by Tuan Le
### June 2018

# 1. Forward and Inverse Kinematic Analysis

## 1.1 Extracting Information from URDF file
From the <span style="color:crimson"> `kr210.urdf.xacro` </span> file, I extracted both x, y, z position and roll, pitch, yaw orientation of each joint of the KR210 robotic arm. Picture below shows a part of the URDF code. I highlight any section that are related to position and orientation information.

```xml
1 <link name="link_1">
2  <inertial>
3    <origin xyz="0 0 0.4" rpy="0 0 0"/>
4    <mass value="${mass1}"/>
5    <inertia ixx="30" ixy="0" ixz="0" iyy="50" iyz="0" izz="50"/>
6  </inertial>
```
Position and orientation of a joint are shown in line 3 as well as a link length of this joint. From these information, I formed a Denavit-Hartenberg table with four parameters (twist angle - **alpha**, joint offset - **a** , link length - **d**, and joint angle - **theta**).

Here is a summary table of position and orientation information according to the <span style="color:crimson"> `kr210.urdf.xacro` </span> file.

Joint name|Parent link|Child link|x (meter)| y (meter)| z (meter)|roll (deg)|pitch (deg)|yaw (deg)|
-|-|-|-|-|-|-|-|-|
Joint 0  | base_footprint | base_link | 0 | 0 | 0 | 0 | 0 | 0
Joint 1  | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0
Joint 2  | link_1 | link_2 | 0.35 | 0 | 0.42 | 0 | 0 | 0
Joint 3  | link_2 | link_3 | 0 | 0 | 1.25 | 0  | 0 | 0
Joint 4  | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0
Joint 5  | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0
Joint 6  | link_5 | link_6 |  0.193| 0 | 0 | 0 | 0 | 0
Gripper joint  | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0


#### Denavit-Hartenberg Table

<p align="center"><img src="./src/RoboND-Kinematics-Project/misc_images/forward_kinematic.png"></p>

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 1.25 | q3
3->4 |  -pi/2 | -0.05 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7

## 1.2 Individual Transformation Matrices of each Joint
#### Creating individual transfromation matrix

#### <span style="color:teal"> a. *Joint 0* to *Joint 1*

#### <span style="color:teal"> b. *Joint 1* to *Joint 2*

#### <span style="color:teal"> c. *Joint 2* to *Joint 3*

#### <span style="color:teal"> d. *Joint 3* to *Joint 4*

#### <span style="color:teal"> e. *Joint 4* to *Joint 5*

#### <span style="color:teal"> f. *Joint 5* to *Joint 6*

#### <span style="color:teal"> g. *Joint 6* to *Joint EE*

## 1.3 Inverse Kinematics Problem

### Inverse Position

### Inverse Orientation

# 2. Project Implementation

### Discussion

### Screenshot of the completed pick and place process
