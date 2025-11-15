#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import sympy as sp
import rospy
from std_msgs.msg import Float64MultiArray

# ---------------- Helpers: degree-based trig for SymPy ----------------
pi = sp.pi
deg = pi / 180

def cosd(x): return sp.cos(deg * x)
def sind(x): return sp.sin(deg * x)

def vex(S):
    """Extract vector from skew-symmetric matrix S."""
    return sp.Matrix([S[2,1], S[0,2], S[1,0]])

# -------------- Build symbolic FK and Jacobian (same as your forward script) --------------
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

    T = T1 * T2 * T3 * T4
    p = T[:3, 3]
    R = T[:3, :3]

    q_vec = sp.Matrix([q1, q2, q3, q4])
    Jv = p.jacobian(q_vec)  # m/deg

    Jw_cols = []
    for i in range(4):
        dR_dqi = sp.diff(R, q_vec[i])      # per degree
        S_i = dR_dqi * R.T
        Jw_cols.append(vex(S_i))           # rad/deg
    Jw = sp.Matrix.hstack(*Jw_cols)

    J = sp.Matrix.vstack(Jv, Jw)           # 6x4

    # lambdify
    vars_J = [l1, l2, l3, l4, q1, q2, q3, q4]
    J_fun = sp.lambdify([vars_J], sp.simplify(J), modules='numpy')
    T_fun = sp.lambdify([vars_J], sp.simplify(T), modules='numpy')

    return J_fun, T_fun

# -------------- Inverse Velocity: damped least squares --------------
def inverse_velocity(J, twist_des, lam=1e-4, qdot_limits_deg=None, W=None,
                     nullspace_target=None, alpha_ns=0.0):
    """
    Compute qdot (deg/s) given twist_des ([vx,vy,vz, wx,wy,wz]) using weighted DLS.
    - J units: [m/deg; rad/deg]
    - twist_des units: [m/s; rad/s]
    """
    m, n = J.shape
    if W is None:
        W = np.eye(n)

    JWJt = J @ W @ J.T
    A = JWJt + (lam**2) * np.eye(m)
    pinv_w = W @ J.T @ np.linalg.inv(A)        # (n x m)

    qdot = pinv_w @ twist_des                  # deg/s

    # Optional nullspace motion
    if nullspace_target is not None and alpha_ns > 0.0:
        N = np.eye(n) - pinv_w @ J
        qdot = qdot + alpha_ns * (N @ nullspace_target)

    # Rate limits
    if qdot_limits_deg is not None:
        qdot_limits_deg = np.asarray(qdot_limits_deg, dtype=float)
        if qdot_limits_deg.size == 1:
            qdot = np.clip(qdot,
                           -qdot_limits_deg.item(),
                           qdot_limits_deg.item())
        else:
            qdot = np.clip(qdot,
                           -qdot_limits_deg,
                           qdot_limits_deg)

    return qdot

def main():
    rospy.init_node('inverse_velocity_node')

    pub = rospy.Publisher('/joint_velocity_cmd', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)   # 10 Hz

    # Build symbolic Jacobian function (once)
    J_fun, T_fun = build_symbolics()

    # -------- Robot + state (degrees) --------
    l1v, l2v, l3v, l4v = 0.0885, 0.140, 0.1329, 0.1105
    qdeg = np.array([-101.3034, 91.5482, 89.0123, 96.5695], dtype=float)  # deg

    # -------- Desired spatial twist (base frame) --------
    # (keep exactly what you had)
    twist_des = np.array([0.02, -0.066, -0.04,   # vx, vy, vz [m/s]
                          0.000, 0.000, 0.050],  # wx, wy, wz [rad/s]
                         dtype=float)

    lam = 1e-4
    qdot_limits = np.array([50, 50, 50, 50], dtype=float)  # deg/s
    W = np.diag([1.0, 1.0, 1.0, 1.0])
    nullspace_target = np.zeros(4)
    alpha_ns = 0.0

    np.set_printoptions(precision=6, suppress=True)

    while not rospy.is_shutdown():
        # Compute Jacobian at current configuration
        J = np.array(J_fun([l1v, l2v, l3v, l4v, *qdeg]), dtype=float)  # 6x4

        # Inverse velocity: joint rates in deg/s
        qdot_deg = inverse_velocity(J, twist_des, lam=lam,
                                    qdot_limits_deg=qdot_limits,
                                    W=W,
                                    nullspace_target=nullspace_target,
                                    alpha_ns=alpha_ns)

        # Publish
        msg = Float64MultiArray()
        msg.data = qdot_deg.tolist()
        pub.publish(msg)

        # Optional: print once in a while
        rospy.loginfo_throttle(1.0, f"qdot_deg published: {qdot_deg}")

        rate.sleep()

if __name__ == "__main__":
    main()

