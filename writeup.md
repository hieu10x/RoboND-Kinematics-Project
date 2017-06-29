## Project: Kinematics Pick & Place

---


[//]: # (Image References)

[fk_demo]: ./misc_images/fk_demo.png
[kuka_diagram]: ./misc_images/kuka_diagram.png
[chart]: ./misc_images/chart.png

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Forward_kinematics demo screenshot:

![forward_kinematics demo screenshot][fk_demo]

Kuka 210 dynamic model

![Kuka 210 dynamic model][kuka_diagram]
Source: from lecture video

From kr210.urdf.xacro file:
```
joint_1: xyz="0 0 0.33"
joint_2: xyz="0.35 0 0.42"
joint_3: xyz="0 0 1.25"
joint_4: xyz="0.96 0 -0.054"
joint_5: xyz="0.54 0 0"
joint_6: xyz="0.193 0 0"
gripper_joint: xyz="0.11 0 0"
```

Deriving the DH parameters:
```python
d1 = 0.33 + 0.42 = 0.75
a1 = 0.35
a3 = -0.054
d4 = 0.96 + 0.54 = 1.5
dG = d7 = 0.193 + 0.11 = 0.303

alpha0 = 0
alpha1 = -pi/2
alpha2 = 0
alpha3 = -pi/2
alpha4 = pi/2
alpha5 = -pi/2
alpha6 = 0

q2 = -pi/2 + q2
```


DH parameter table

i | `alpha[i-1]` | `a[i-1]` | `d[i]`  | `theta[i]`
--| ------------ | -------- | ------- | -----------
1 | `0`          | `0`      | `0.75`  | `q1`
2 | `-pi/2`      | `0.35`   | `0`     | `q2 - pi/2`
3 | `0`          | `0`      | `0`     | `q3`
4 | `-pi/2`      | `-0.054` | `1.5`   | `q4`
5 | `pi/2`       | `0`      | `0`     | `q5`
6 | `-pi/2`      | `0`      | `0`     | `q6`
7 | `0`          | `0`      | `0.303` | `0`


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

For each joint, the DH transform is the multiplication of 4 individual transforms:

```python
T = R_x(alpha[i-1]) * D_x(a[i-1]) * R_z(theta[i]) * D_z(d[i])
  = Matrix([[cos(q)                , -sin(q)               , 0               , a_1               ],
            [sin(q)*cos(alpha[i-1]), cos(q)*cos(alpha[i-1]), -sin(alpha[i-1]), -sin(alpha[i-1])*d],
            [sin(q)*sin(alpha[i-1]), cos(q)*sin(alpha[i-1]), cos(alpha[i-1]) , cos(alpha[i-1])*d ],
            [0                     , 0                     , 0               , 1                 ]
           ])
```

Homogeneous transformation matrices for each joint:

```python
T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0   ],
               [sin(q1), cos(q1),  0, 0   ],
               [0      ,       0,  1, 0.75],
               [0      ,       0,  0, 1   ]])

T1_2 = Matrix([[sin(q2), cos(q2) , 0, 0.35],
               [0      , 0       , 1, 0   ],
               [cos(q2), -sin(q2), 0, 0   ],
               [0      , 0       , 0, 1   ]])

T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25],
               [sin(q3), cos(q3) , 0, 0   ],
               [0      , 0       , 1, 0   ],
               [0      , 0       , 0, 1   ]])

T3_4 = Matrix([[cos(q4) , -sin(q4), 0, -0.054],
               [0       , 0       , 1, 1.5   ],
               [-sin(q4), -cos(q4), 0, 0     ],
               [0       , 0       , 0, 1     ]])

T4_5 = Matrix([[cos(q5), -sin(q5), 0 , 0],
               [0      , 0       , -1, 0],
               [sin(q5), cos(q5) , 0 , 0],
               [0      , 0       , 0 , 1]])

T5_6 = Matrix([[cos(q6) , -sin(q6), 0, 0],
               [0       , 0       , 1, 0],
               [-sin(q6), -cos(q6), 0, 0],
               [0       , 0       , 0, 1]])

T6_G = Matrix([[1, 0, 0, 0    ],
               [0, 1, 0, 0    ],
               [0, 0, 1, 0.303],
               [0, 0, 0, 1    ]])
```

The generalized homogeneous transform between base_link and gripper_link:

```python
T_total = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G * (rot_z(pi) * rot_y(-pi/2))
        = Matrix([
[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)],
[-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)],
[                                    -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                               -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[                                                                                       0,                                                                                                                                                            0,                                                                                                                                                            0,                                                                                                                                                                            1]])
```

Notice that the factor `(rot_z(pi) * rot_y(-pi/2)` is due to the difference between the URDF and the DH coordinates.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The last 3 joints of our robot form a **spherical joint** with `joint_5` being the **wrist center**. The problem can be decoupled into Inverse Position and Inverse Orientation problems. The position of the wrist center `W_G` is governed by the first 3 joints `q1, q2, q3`. The wrist center also has a simple relation to the position of the end-effector. The orientation of the end-effector is governed by the last 3 joints `q4, q5, q6`.


The inputs of the Inverse Kinematics problem are the position (p_x, p_y, p_z) and orientation (n_x, n_y, n_z) of the end-effector. Note that n is the orthonormal vector representing the end-effector orientation along the Z axis of the local coordinate frame.

###### Position

The position of the wrist center can be derived:
```python
x = p_x - d6 * n_x
y = p_y - d6 * n_y
z = p_z - d6 * n_z
```

Position of `joint_4` is also the position of `joint_5`. The generalized homogeneous transform between `base_link` and `link_4`:


###### Equation 1
```python
q1 = atan2(y,x)  # Eq 1
```

###### Equation 2
```python
T0_4 = T0_1 * T1_2 * T2_3 * T3_4
     = Matrix([
                [sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4),  sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)],
                [sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1), -sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4), sin(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)],
                [                          cos(q4)*cos(q2 + q3),                           -sin(q4)*cos(q2 + q3),        -sin(q2 + q3),          -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
                [                                             0,                                               0,                    0,                                                                     1]])
```

From the homogeneous transform, we can extract the translation transform which is the position of `W_G`

```python
x = (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)
y = (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)
z = -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75

```

Let
```python
r = 1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3)
s = -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3)
```
The square of the distance between joint_2 and joint_4 can be calculated as belows:
```python
distance = r**2 + s**2
         = -3.75*sin(q3) - 0.135*cos(q3) + 3.815416
```
or
```python
distance = (x - a1*cos(q1))**2 + (y - a1*sin(q1))**2 + (z - d1)**2
```
Comparing the two above equations gives:
```python
-3.75*sin(q3) - 0.135*cos(q3) + 3.815416 = (x - 0.35*cos(q1))**2 + (y - 0.35*sin(q1))**2 + (-z + 0.75)**2  # Eq 2
```

By substitude `x, y, z` and `q1`, we can yield `q3`

###### Equation 3
We have
```python
s = -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3)
```

`s` can also be calculated by `z - d1`

```python
-1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) = z - 0.75  # Eq 3
```
By substitude `z` and `q3`, we can yield `q2`

###### Orientation

The rotation matrix from `base_link` and `gripper_link`:
```python
Rrpy = R0_6 = R0_3 * R3_6  # chain rule 0 to 3, 3 to 6
```
Multiplying both sides with R0_3.inv() yields:
```python
R3_6 = R0_3.inv() * Rrpy
```
On the left hand side:
`R0_3` is given by:
```python
R0_3 = Matrix([
               [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
               [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
               [        cos(q2 + q3),        -sin(q2 + q3),        0]])
```
With `q1, q2, q3` substituded, `R0_3` contains no variables. `Rrpy` can be yielded from the input `roll, pitch, yaw`, notice the difference between URDF and DH coordinates.
On the right hand side:
```python
R3_6 = Matrix([
                [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
                [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
                [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
```
 Solving the system equations gives `q4, q5, q6`. Notice that the element in row 3, column 3, `cos(q5)` only has 1 variable `q5`. We can easily solve the equation system.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Given the analysis from the above section, the implement is straight-forward. To solve the equations, I use the numerical method with the help of the `nsolve()` function of `sympy` package. See code for more details.

One problem I encountered was the values of the last 3 joints fluctuated greatly due to multiple roots. My approach is listing all the solutions and choose the one that has least "distance" from the previous state.

The result from one random run is recorded. See https://docs.google.com/spreadsheets/d/1nuDoi4nu4353hvqs8Wjm81k6OdFrh6Op6l5_owfQJek/edit?usp=sharing . The result has one largest off point where the error is ~0.000005 m.

![Error plot][chart]