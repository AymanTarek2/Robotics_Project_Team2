#!/usr/bin/env python3
import numpy as np

def fk_4dof(q1, q2, q3, q4, l1, l2, l3, l4):
    """Compute the end-effector position and ZYX Euler angles (deg)."""

    # Convert degrees to radians
    d2r = np.pi / 180
    q1, q2, q3, q4 = q1 * d2r, q2 * d2r, q3 * d2r, q4 * d2r

    # Define shorthand functions
    c = np.cos
    s = np.sin
    pi = np.pi

    # --- Transformation Matrices (same as MATLAB) ---
    T1 = np.array([
        [c(q1), -s(q1)*c(pi/2),  s(q1)*s(pi/2),  0],
        [s(q1),  c(q1)*c(pi/2), -c(q1)*s(pi/2),  0],
        [0,           s(pi/2),         c(pi/2),  l1],
        [0,                 0,               0,   1]
    ])

    T2 = np.array([
        [c(q2+pi/2), -s(q2+pi/2)*c(pi),  s(q2+pi/2)*s(pi),  l2*c(q2+pi/2)],
        [s(q2+pi/2),  c(q2+pi/2)*c(pi), -c(q2+pi/2)*s(pi),  l2*s(q2+pi/2)],
        [0,                     s(pi),              c(pi),             0],
        [0,                         0,                 0,              1]
    ])

    T3 = np.array([
        [c(q3), -s(q3)*c(pi),  s(q3)*s(pi),  l3*c(q3)],
        [s(q3),  c(q3)*c(pi), -c(q3)*s(pi),  l3*s(q3)],
        [0,          s(pi),          c(pi),          0],
        [0,              0,              0,          1]
    ])

    T4 = np.array([
        [c(q4), -s(q4)*c(0),  s(q4)*s(0),  l4*c(q4)],
        [s(q4),  c(q4)*c(0), -c(q4)*s(0),  l4*s(q4)],
        [0,         s(0),          c(0),          0],
        [0,             0,              0,          1]
    ])

    # Final transformation
    T_final = T1 @ T2 @ T3 @ T4

    # --- Extract EE position ---
    x, y, z = T_final[0, 3], T_final[1, 3], T_final[2, 3]

    # --- Orientation matrix ---
    R = T_final[0:3, 0:3]
    sy = np.sqrt(R[2, 1]**2 + R[2, 2]**2)

    # --- Euler angles (ZYX sequence: yaw, pitch, roll) ---
    if sy > 1e-9:
        thetay = np.arctan2(-R[2, 0], sy)  # pitch
        thetax = np.arctan2(R[2, 1], R[2, 2])  # roll
        thetaz = np.arctan2(R[1, 0], R[0, 0])  # yaw
    else:
        # Gimbal lock case
        thetay = np.arctan2(-R[2, 0], 0)
        thetax = 0
        thetaz = np.arctan2(-R[0, 1], R[1, 1])

    # Convert to degrees
    thetax = np.degrees(thetax)
    thetay = np.degrees(thetay)
    thetaz = np.degrees(thetaz)

    return (x, y, z), (thetax, thetay, thetaz)


if __name__ == "__main__":
    # ---- Link lengths ----
    l1, l2, l3, l4 = 0.0885, 0.140, 0.1329, 0.1105

    # ---- Given joint angles (deg) ----
    q1, q2, q3, q4 = -101.3034, 91.5482, 89.0123, 96.5695

    # ---- Compute FK ----
    pos, euler = fk_4dof(q1, q2, q3, q4, l1, l2, l3, l4)

    # ---- Display results ----
    print("End-Effector Position:")
    print(f"x = {pos[0]:.4f} m")
    print(f"y = {pos[1]:.4f} m")
    print(f"z = {pos[2]:.4f} m\n")

    print("End-Effector Orientation (Euler Angles - ZYX):")
    print(f"roll  (theta_x) = {euler[0]:.4f}°")
    print(f"pitch (theta_y) = {euler[1]:.4f}°")
    print(f"yaw   (theta_z) = {euler[2]:.4f}°")

