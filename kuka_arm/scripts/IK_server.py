#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
import numpy as np
# from mpmath.calculus.optimization import MNewton

def my_nsolve(eq, st):
    for i in range(20):
        try:
            return nsolve(eq, st - np.pi/20 + i * np.pi/5, x1=(st + np.pi/20 + i * np.pi/5))
        except ValueError:
            pass
    return 0.0

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        rospy.loginfo("Entering main calc section")


        # Define DH param symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Joint angle symbols
        x, y, z = symbols('x y z')
        r, s = symbols('r s')

        # Modified DH params
        dh = {
            alpha0: 0    , a0: 0     , d1: 0.75 ,
            alpha1: -pi/2, a1: 0.35  , d2: 0    , q2: -pi/2 + q2,
            alpha2: 0    , a2: 1.25  , d3: 0    ,
            alpha3: -pi/2, a3: -0.054, d4: 1.5  ,
            alpha4:  pi/2, a4: 0     , d5: 0    ,
            alpha5: -pi/2, a5: 0     , d6: 0    ,
            alpha6: 0    , a6: 0     , d7: 0.303, q7: 0,
        }


        # Define Modified DH Transformation matrix
        def dh_transform(al_1, a_1, d, q):
            return (Matrix([[cos(q)          , -sin(q)         , 0         , a_1         ],
                            [sin(q)*cos(al_1), cos(q)*cos(al_1), -sin(al_1), -sin(al_1)*d],
                            [sin(q)*sin(al_1), cos(q)*sin(al_1), cos(al_1) , cos(al_1)*d ],
                            [0               , 0               , 0         , 1           ]
                           ]))

        def rot_z(q):
            return Matrix([[ cos(q), -sin(q), 0, 0 ],
                           [ sin(q),  cos(q), 0, 0 ],
                           [      0,       0, 1, 0 ],
                           [      0,       0, 0, 1 ]])

        def rot_y(q):
            return Matrix([[  cos(q), 0, sin(q), 0 ],
                           [       0, 1,      0, 0 ],
                           [ -sin(q), 0, cos(q), 0 ],
                           [       0, 0,      0, 1 ]])

        def rot_x(q):
            return Matrix([[       1,      0,       0, 0],
                           [       0, cos(q), -sin(q), 0],
                           [       0, sin(q),  cos(q), 0],
                           [       0,      0,       0, 1]])

        # To speed up calculation, the matrices are computed
        # and the results are used in the following steps
        #
        # T0_1 = dh_transform(alpha0, a0, d1, q1)
        # T0_1 = T0_1.subs(dh)
        # T1_2 = dh_transform(alpha1, a1, d2, q2)
        # T1_2 = T1_2.subs(dh)
        # T2_3 = dh_transform(alpha2, a2, d3, q3)
        # T2_3 = T2_3.subs(dh)
        # T3_4 = dh_transform(alpha3, a3, d4, q4)
        # T3_4 = T3_4.subs(dh)
        # T4_5 = dh_transform(alpha4, a4, d5, q5)
        # T4_5 = T4_5.subs(dh)
        # T5_6 = dh_transform(alpha5, a5, d6, q6)
        # T5_6 = T5_6.subs(dh)


        # # Composition of homogeneous tranforms
        # T0_2 = simplify(T0_1 * T1_2)
        # T0_3 = simplify(T0_2 * T2_3)
        # print('T0_3 =', T0_3)
        # T0_4 = simplify(T0_3 * T3_4)
        # print('T0_4 =', T0_4)

        # T3_6 = simplify(T3_4 * T4_5 * T5_6)
        # print('T3_6 = ', T3_6)


        T0_3 = Matrix([
                        [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1), (1.25*sin(q2) + 0.35)*cos(q1)],
                        [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1), (1.25*sin(q2) + 0.35)*sin(q1)],
                        [        cos(q2 + q3),        -sin(q2 + q3),        0,           1.25*cos(q2) + 0.75],
                        [                   0,                    0,        0,                             1]])

        T0_4 = Matrix([
                        [sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4),  sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)],
                        [sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1), -sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4), sin(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)],
                        [                          cos(q4)*cos(q2 + q3),                           -sin(q4)*cos(q2 + q3),        -sin(q2 + q3),          -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
                        [                                             0,                                               0,                    0,                                                                     1]])
        T3_6 = Matrix([
                        [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.054],
                        [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),    1.5],
                        [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),      0],
                        [                                         0,                                          0,                0,      1]])


        # Create individual transformation matrices
        # with r, s are as in the example
        r = simplify(T0_4[0,3] / cos(q1) - a1)  # (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3))
        r = r.subs({a1: dh[a1]})
        # print('r', r)

        s = T0_4[2, 3] - d1  # -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3)
        s = s.subs({d1: dh[d1]})
        # print('s', s)
        eq2_lhs = trigsimp(r**2 + s**2)
        # print('eq1_lhs', eq2_lhs)
        eq2_rhs = simplify((x - a1*cos(q1))**2 + (y - a1*sin(q1))**2 + (z - d1)**2)
        eq2_rhs = eq2_rhs.subs({a1: dh[a1], d1: dh[d1]})
        # print('eq2_rhs', eq2_rhs)

        # Equation 1, solve for q1
        # eq1 = q1 - atan2(y, x)

        # Equation 2, solve for q3
        eq2 = eq2_lhs - eq2_rhs

        # Equation 3, solve for q2
        eq3 = s - (z - d1)
        eq3 = eq3.subs({d1: dh[d1]})



        # Solve for q4, q5, q6
        R0_3_inv = T0_3[:3,:3].inv()
        R3_6_lhs = T3_6[:3, :3]
        # R3_6_lhs = (T3_6 * (rot_z(pi) * rot_y(-pi/2)))[:3, :3]

        def cap_angle(q):
            while q > np.pi:
                q -= 2*np.pi
            while q < -np.pi:
                q += 2*np.pi
            return q
            
        def check_solution(a, b):
            for i in range(3):
                for j in range(3):
                    if abs(a[i,j] - b[i,j]) > 0.0001:
                        return False
            return True

        def solve_rot_3_6(R3_6_lhs, R3_6_rhs, old_values):
            q5_val_t = cap_angle(my_nsolve(R3_6_lhs[1,2] - R3_6_rhs[1,2], old_values[4]))
            solutions = []
            for q5_val in (q5_val_t, -q5_val_t):
                q4_val_t = cap_angle(my_nsolve(R3_6_lhs[0,2].subs({q5: q5_val}) - R3_6_rhs[0,2], old_values[3]))
                for q4_val in (q4_val_t, -q4_val_t):
                    q6_val_t = cap_angle(my_nsolve(R3_6_lhs[1,0].subs({q5: q5_val}) - R3_6_rhs[1,0], old_values[5]))
                    for q6_val in (q6_val_t, -q6_val_t):
                        if check_solution(R3_6_lhs.subs({q4: q4_val, q5: q5_val, q6: q6_val}), R3_6_rhs):
                            solutions.append((q4_val, q5_val, q6_val))
            if len(solutions) == 0:
                rospy.logerr("ERROR!!! Cannot solve rot 3 6")
                return .0, .0, .0
            else:
                errs = [(s[0] - old_values[3])**2 + (s[1] - old_values[4])**2 + (s[2] - old_values[5])**2 for s in solutions]
                return solutions[np.argmin(errs)]
            return 0, 0, 0

        old_values = (0,0,0,0,0,0)

        for i in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[i].position.x
            py = req.poses[i].position.y
            pz = req.poses[i].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[i].orientation.x, req.poses[i].orientation.y,
                 req.poses[i].orientation.z, req.poses[i].orientation.w])


            # Calculate joint angles using Geometric IK method
            R_rpy = (rot_z(yaw) * rot_y(pitch) * rot_x(roll) * (rot_z(pi) * rot_y(-pi/2)))[:3, :3]

            ee_length = d7.evalf(subs=dh)
            c_sub = {x: px - ee_length * R_rpy[0, 2],
                     y: py - ee_length * R_rpy[1, 2],
                     z: pz - ee_length * R_rpy[2, 2]}

            # eq1_t = eq1.subs(c_sub)
            # # print('eq1_t', eq1_t)
            # theta1 = my_nsolve(eq1_t, old_values[0])
            theta1 = atan2(y,x).evalf(subs=c_sub)

            eq2_t = eq2.subs(c_sub).subs({q1: theta1})
            # print('eq2_t', eq2_t)
            theta3 = my_nsolve(eq2_t, old_values[2])

            eq3_t = eq3.subs(c_sub).subs({q3: theta3})
            # print('eq3_t', eq3_t)
            theta2 = my_nsolve(eq3_t, old_values[1])

            R3_6_rhs = R0_3_inv * R_rpy
            R3_6_rhs = R3_6_rhs.subs({q1:theta1, q2:theta2, q3:theta3})
            # print('R3_6_rhs', R3_6_rhs)
            theta4, theta5, theta6 = solve_rot_3_6(R3_6_lhs, R3_6_rhs, old_values)
            old_values = (theta1, theta2, theta3, theta4, theta5, theta6)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]

            # # error calculation
            # P_fw = Matrix([
            #                 [-0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)],
            #                 [-0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)],
            #                 [                                              -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75]])
            # P_fw = P_fw.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
            # print('>>> I:', px, py, pz)
            # print('>>> O:', P_fw[0,0], P_fw[1,0], P_fw[2,0])


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
