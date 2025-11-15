#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import sympy as sp
import rospy
from std_msgs.msg import Float64MultiArray

# ---------------- Helpers: degree-based trig for SymPy ----------------
pi = sp.pi
deg = pi/180

def cosd(x): return sp.cos(deg*x)
def sind(x): return sp.sin(deg*x)

def vex(S):
    """Extract vector from skew-symmetric matrix S."""
    return sp.Matrix([S[2,1], S[0,2], S[1,0]])

# -------------- Build symbolic FK and Jacobian (same as above) --------------
def build_symbolics():
    l1, l2, l3, l4 = sp.symbols('l1 l2 l3 l4', real=True)
    q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4', real=True)    # degrees

    T1 = sp.Matrix([
        [ cosd(q1), -sind(q1)*cosd(90),  sind(q1)*sind(90),  0],
        [ sind(q1),  cosd(q1)*cosd(90), -cosd(q1)*sind(90),  0],
        [        0,             sind(90),          cosd(90), l1],
        [        0,                    0,                 0,  1]
    ])

    T2 = sp.Matrix([
        [ cosd(q2+90), -sind(q2+90)*cosd(180),  sind(q2+90)*sind(180),  l2*cosd(q2+90)],
        [ sind(q2+90),  cosd(q2+90)*cosd(180), -cosd(q2+90)*sind(180),  l2*sind(q2+90)],
        [           0,               sind(180),               cosd(180),             0],
        [           0,                      0,                      0,                1]
    ])

    T3 = sp.Matrix([
        [ cosd(q3), -sind(q3)*cosd(180),  sind(q3)*sind(180),  l3*cosd(q3)],
        [ sind(q3),  cosd(q3)*cosd(180), -cosd(q3)*sind(180),  l3*sind(q3)],
        [        0,               sind(180),           cosd(180),          0],
        [        0,                      0,                   0,            1]
    ])

    T4 = sp.Matrix([
        [ cosd(q4), -sind(q4)*cosd(0),  sind(q4)*sind(0),  l4*cosd(q4)],
        [ sind(q4),  cosd(q4)*cosd(0), -cosd(q4)*sind(0),  l4*sind(q4)],
        [        0,             sind(0),           cosd(0),          0],
        [        0,                  0,                0,            1]
    ])

    T = T1*T2*T3*T4
    p = T[:3, 3]
    R = T[:3, :3]

    q_vec = sp.Matrix([q1, q2, q3, q4])
    Jv = p.jacobian(q_vec)

    Jw_cols = []
    for i in range(4):
        dR_dqi = sp.diff(R, q_vec[i])
        S_i = dR_dqi * R.T
        Jw_cols.append(vex(S_i))
    Jw = sp.Matrix.hstack(*Jw_cols)

    J = sp.Matrix.vstack(Jv, Jw)

    vars_J = [l1, l2, l3, l4, q1, q2, q3, q4]
    J_fun = sp.lambdify([vars_J], sp.simplify(J), modules='numpy')

    return J_fun, T  # T is symbolic, in case you want FK later

# -------------- Global state --------------
last_qdot_deg = np.zeros(4)
Jnum_global = None

def joint_velocity_callback(msg):
    global last_qdot_deg, Jnum_global

    # qdot_deg from inverse node (deg/s)
    data = np.array(msg.data, dtype=float)
    if data.size != 4:
        rospy.logwarn("Received joint_velocity_cmd of wrong size (expected 4).")
        return

    last_qdot_deg = data

    # Compute twist = J * qdot (units consistent with your forward code)
    if Jnum_global is None:
        rospy.logwarn("Jacobian not initialized yet.")
        return

    twist = Jnum_global @ last_qdot_deg  # [vx, vy, vz, wx, wy, wz]^T
    vx, vy, vz, wx, wy, wz = twist

    rospy.loginfo("\n=== Forward velocity from subscribed qdot ===")
    rospy.loginfo(f"qdot_deg = {last_qdot_deg}")
    rospy.loginfo(f"v = [{vx:.6f}  {vy:.6f}  {vz:.6f}] m/s")
    rospy.loginfo(f"w = [{wx:.6f}  {wy:.6f}  {wz:.6f}] rad/s")

def main():
    global Jnum_global

    rospy.init_node('forward_velocity_node')

    # Build symbolic J once
    J_fun, _ = build_symbolics()

    # Same robot and configuration as in inverse node
    l1v, l2v, l3v, l4v = 0.0885, 0.140, 0.1329, 0.1105
    q1v, q2v, q3v, q4v = -101.3034, 91.5482, 89.0123, 96.5695   # degrees

    # Numeric Jacobian at that configuration
    Jnum_global = np.array(J_fun([l1v, l2v, l3v, l4v,
                                  q1v, q2v, q3v, q4v]), dtype=float)

    rospy.loginfo("Jacobian J (m/deg; rad/deg):\n%s", Jnum_global)

    # Subscriber: joint velocities from inverse node
    rospy.Subscriber('/joint_velocity_cmd', Float64MultiArray, joint_velocity_callback)

    rospy.loginfo("forward_velocity_node is running and waiting for /joint_velocity_cmd ...")
    rospy.spin()

if __name__ == "__main__":
    main()

