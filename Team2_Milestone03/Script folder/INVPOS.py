#!/usr/bin/env python3
import numpy as np

def wrap_to_pi(q):
    """Wrap angles to [-pi, pi]."""
    return (q + np.pi) % (2 * np.pi) - np.pi

def fk_4dof(q, l1, l2, l3, l4):
    """Forward kinematics for the 4-DOF chain using the same T1..T4 as in MATLAB."""
    q1, q2, q3, q4 = q

    # Precompute constants
    pi = np.pi
    c = np.cos
    s = np.sin

    T1 = np.array([
        [ c(q1), -s(q1)*c(pi/2),  s(q1)*s(pi/2),  0.0],
        [ s(q1),  c(q1)*c(pi/2), -c(q1)*s(pi/2),  0.0],
        [   0.0,          s(pi/2),       c(pi/2),  l1 ],
        [   0.0,             0.0,          0.0,   1.0]
    ])

    T2 = np.array([
        [ c(q2+pi/2), -s(q2+pi/2)*c(pi),  s(q2+pi/2)*s(pi),  l2*c(q2+pi/2)],
        [ s(q2+pi/2),  c(q2+pi/2)*c(pi), -c(q2+pi/2)*s(pi),  l2*s(q2+pi/2)],
        [        0.0,             s(pi),              c(pi),           0.0],
        [        0.0,              0.0,               0.0,            1.0]
    ])

    T3 = np.array([
        [ c(q3), -s(q3)*c(pi),  s(q3)*s(pi),  l3*c(q3)],
        [ s(q3),  c(q3)*c(pi), -c(q3)*s(pi),  l3*s(q3)],
        [   0.0,        s(pi),        c(pi),       0.0],
        [   0.0,         0.0,         0.0,        1.0]
    ])

    T4 = np.array([
        [ c(q4), -s(q4)*c(0.0),  s(q4)*s(0.0),  l4*c(q4)],
        [ s(q4),  c(q4)*c(0.0), -c(q4)*s(0.0),  l4*s(q4)],
        [   0.0,        s(0.0),        c(0.0),       0.0],
        [   0.0,         0.0,         0.0,        1.0]
    ])

    T = T1 @ T2 @ T3 @ T4
    p = T[:3, 3]
    return T, p

def main():
    # ---------------- Desired End-Effector Position ----------------
    xd = 0.05   # meters
    yd = 0.25
    zd = 0.20
    pos_des = np.array([xd, yd, zd], dtype=float)

    # ---------------- Robot Parameters ----------------
    l1 = 0.0885
    l2 = 0.140
    l3 = 0.1329
    l4 = 0.1105

    # ---------------- Initial Guess (radians) ----------------
    q = np.deg2rad(np.array([3.0, 6.0, 6.0, 9.0], dtype=float))  # initial guess
    max_iter = 200                  # maximum iterations
    tol = 1e-4                      # position error tolerance (m)
    alpha = 0.6                     # step size (0 < alpha ≤ 1)
    max_step = np.deg2rad(5.0)      # limit per update (radians)

    # ---------------- Iterative Numerical IK ----------------
    for k in range(1, max_iter + 1):
        # ---- Forward Kinematics ----
        _, p = fk_4dof(q, l1, l2, l3, l4)

        # ---- Position Error ----
        e = pos_des - p
        err_norm = np.linalg.norm(e)

        # ---- Display iteration progress ----
        print(f"Iter {k:3d} | Error = {err_norm:.6f} m")

        # ---- Convergence check ----
        if err_norm < tol:
            print(f"Converged after {k} iterations.")
            break

        # ---- Numerical Jacobian ----
        dq_eps = 1e-4  # small radian perturbation
        Jv = np.zeros((3, 4), dtype=float)
        for i in range(4):
            q_pert = q.copy()
            q_pert[i] += dq_eps
            _, pp = fk_4dof(q_pert, l1, l2, l3, l4)
            Jv[:, i] = (pp - p) / dq_eps  # m/rad

        # ---- Compute update ----
        dq_update = alpha * (np.linalg.pinv(Jv) @ e)  # scaled step

        # limit movement per joint
        dq_update = np.clip(dq_update, -max_step, max_step)

        # ---- Update joint angles ----
        q = q + dq_update
        q = wrap_to_pi(q)  # keep angles within [-pi, pi]

    # Final FK for reporting
    _, p_final = fk_4dof(q, l1, l2, l3, l4)
    err_final = np.linalg.norm(pos_des - p_final)

    # ---------------- Display Results ----------------
    q_deg = np.rad2deg(q)
    print("\nFinal Joint Angles (deg):")
    print(f"q1 = {q_deg[0]:.4f}°")
    print(f"q2 = {q_deg[1]:.4f}°")
    print(f"q3 = {q_deg[2]:.4f}°")
    print(f"q4 = {q_deg[3]:.4f}°")

    print("\nFinal End-Effector Position:")
    print(f"x = {p_final[0]:.4f}, y = {p_final[1]:.4f}, z = {p_final[2]:.4f}")
    print(f"Final Error Norm = {err_final:.6f} m")

if __name__ == "__main__":
    main()

