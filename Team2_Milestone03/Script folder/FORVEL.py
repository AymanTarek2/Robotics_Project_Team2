#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Forward velocity kinematics (symbolic differentiation approach)
- Matches the MATLAB structure you provided (angles in DEGREES in the DH blocks).
- Jv = ∂p/∂q, Jw from S_i = (∂R/∂q_i) * R^T, vex(S_i)
- Units: Jv in m/deg, Jw in rad/deg; twist = J * qdot (deg/s) -> [m/s; rad/s]
"""
import numpy as np
import sympy as sp

# ---------------- Helpers: degree-based trig for SymPy ----------------
pi = sp.pi
deg = pi/180

def cosd(x): return sp.cos(deg*x)
def sind(x): return sp.sin(deg*x)
def atan2d(y, x): return sp.atan2(y, x)/deg

def vex(S):
    """Extract vector from skew-symmetric matrix S = [[0,-wz,wy],[wz,0,-wx],[-wy,wx,0]]."""
    return sp.Matrix([S[2,1], S[0,2], S[1,0]])

def main():
    # ---------------- Symbols ----------------
    l1, l2, l3, l4 = sp.symbols('l1 l2 l3 l4', real=True)
    q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4', real=True)         # degrees
    q1d, q2d, q3d, q4d = sp.symbols('q1d q2d q3d q4d', real=True) # deg/s

    # ---------------- DH-style transforms (angles in degrees, exactly like MATLAB) ----------------
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
    p = T[:3, 3]          # position
    R = T[:3, :3]         # rotation

    # ---------------- Jacobian via differentiation ----------------
    q_vec = sp.Matrix([q1, q2, q3, q4])

    # Jv = dp/dq  (m/deg)
    Jv = p.jacobian(q_vec)

    # Jw columns: vex( dR/dq_i * R^T )  (rad/deg)
    Jw_cols = []
    for i in range(4):
        dR_dqi = sp.diff(R, q_vec[i])      # per degree
        S_i = dR_dqi * R.T                 # skew matrix
        w_i = vex(S_i)                     # rad per deg
        Jw_cols.append(w_i)
    Jw = sp.Matrix.hstack(*Jw_cols)

    # Full spatial Jacobian (base frame)
    J = sp.Matrix.vstack(Jv, Jw)

    # ---------------- Map joint rates (deg/s) to spatial twist ----------------
    qdot = sp.Matrix([q1d, q2d, q3d, q4d])     # deg/s
    twist = J * qdot                           # [vx vy vz wx wy wz]^T ; v in m/s, w in rad/s

    # ---------------- Create numeric functions ----------------
    # Vars order for evaluation (match MATLAB vector)
    vars_all = [l1, l2, l3, l4, q1, q2, q3, q4, q1d, q2d, q3d, q4d]
    vars_J = [l1, l2, l3, l4, q1, q2, q3, q4]

    twist_fun = sp.lambdify([vars_all], sp.simplify(twist), modules='numpy')
    J_fun     = sp.lambdify([vars_J],  sp.simplify(J),     modules='numpy')
    T_fun     = sp.lambdify([vars_J],  sp.simplify(T),     modules='numpy')

    # ---------------- Numeric evaluation (your values) ----------------
    l1v, l2v, l3v, l4v = 0.0885, 0.140, 0.1329, 0.1105
    q1v, q2v, q3v, q4v = -101.3034, 91.5482, 89.0123, 96.5695   # degrees

    # joint rates (deg/s)
    q1dv, q2dv, q3dv, q4dv = -4.477675, 2.306787, 2.771328, 4.811676

    tw = np.array(twist_fun([l1v, l2v, l3v, l4v, q1v, q2v, q3v, q4v, q1dv, q2dv, q3dv, q4dv]), dtype=float).reshape(-1)
    Jnum = np.array(J_fun([l1v, l2v, l3v, l4v, q1v, q2v, q3v, q4v]), dtype=float)

    # Forward kinematics numeric
    Tn = np.array(T_fun([l1v, l2v, l3v, l4v, q1v, q2v, q3v, q4v]), dtype=float)
    xn, yn, zn = Tn[0, 3], Tn[1, 3], Tn[2, 3]
    Rn = Tn[:3, :3]

    # Manual ZYX Euler (roll = x, pitch = y, yaw = z), degrees
    sy_n = np.sqrt(Rn[2,1]**2 + Rn[2,2]**2)
    if sy_n > 1e-9:
        thetay = np.degrees(np.arctan2(-Rn[2,0], sy_n))
        thetax = np.degrees(np.arctan2(Rn[2,1], Rn[2,2]))
        thetaz = np.degrees(np.arctan2(Rn[1,0], Rn[0,0]))
    else:
        thetay = np.degrees(np.arctan2(-Rn[2,0], 0.0))
        thetax = 0.0
        thetaz = np.degrees(np.arctan2(-Rn[0,1], Rn[1,1]))

    # ---------------- Display ----------------
    print('=== Forward Kinematics (numeric) ===')
    print(f'x = {xn:.4f} m')
    print(f'y = {yn:.4f} m')
    print(f'z = {zn:.4f} m')

    print('\nEuler ZYX (deg):')
    print(f'roll  (theta_x) = {thetax:.4f} deg')
    print(f'pitch (theta_y) = {thetay:.4f} deg')
    print(f'yaw   (theta_z) = {thetaz:.4f} deg')

    print('\n=== Jacobian from differentiation (numeric) ===')
    # rows: [Jv; Jw], Jv in m/deg, Jw in rad/deg
    np.set_printoptions(precision=6, suppress=True)
    print(Jnum)

    vx, vy, vz, wx, wy, wz = tw
    print('\n=== End-Effector Spatial Velocity (base frame) ===')
    print(f'v = [{vx:.6f}  {vy:.6f}  {vz:.6f}] m/s')
    print(f'w = [{wx:.6f}  {wy:.6f}  {wz:.6f}] rad/s')

if __name__ == "__main__":
    main()

