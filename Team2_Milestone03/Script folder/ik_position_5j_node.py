#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# --------- your IK core (unchanged math) ----------
def wrap_to_pi(q):
    return (q + np.pi) % (2*np.pi) - np.pi

def fk_4dof(q, l1, l2, l3, l4):
    q1, q2, q3, q4 = q
    pi = np.pi; c = np.cos; s = np.sin
    T1 = np.array([[ c(q1), -s(q1)*c(pi/2),  s(q1)*s(pi/2),  0.0],
                   [ s(q1),  c(q1)*c(pi/2), -c(q1)*s(pi/2),  0.0],
                   [   0.0,          s(pi/2),       c(pi/2),  l1 ],
                   [   0.0,             0.0,          0.0,   1.0]])
    T2 = np.array([[ c(q2+pi/2), -s(q2+pi/2)*c(pi),  s(q2+pi/2)*s(pi),  l2*c(q2+pi/2)],
                   [ s(q2+pi/2),  c(q2+pi/2)*c(pi), -c(q2+pi/2)*s(pi),  l2*s(q2+pi/2)],
                   [        0.0,             s(pi),              c(pi),           0.0],
                   [        0.0,              0.0,               0.0,            1.0]])
    T3 = np.array([[ c(q3), -s(q3)*c(pi),  s(q3)*s(pi),  l3*c(q3)],
                   [ s(q3),  c(q3)*c(pi), -c(q3)*s(pi),  l3*s(q3)],
                   [   0.0,        s(pi),        c(pi),       0.0],
                   [   0.0,         0.0,         0.0,        1.0]])
    T4 = np.array([[ c(q4), -s(q4)*c(0.0),  s(q4)*s(0.0),  l4*c(q4)],
                   [ s(q4),  c(q4)*c(0.0), -c(q4)*s(0.0),  l4*s(q4)],
                   [   0.0,        s(0.0),        c(0.0),       0.0],
                   [   0.0,         0.0,         0.0,        1.0]])
    T = T1 @ T2 @ T3 @ T4
    return T, T[:3,3]

def numeric_Jv(q, l1, l2, l3, l4, eps=1e-4):
    _, p = fk_4dof(q, l1,l2,l3,l4)
    Jv = np.zeros((3,4))
    for i in range(4):
        qp = q.copy(); qp[i] += eps
        _, pp = fk_4dof(qp, l1,l2,l3,l4)
        Jv[:,i] = (pp - p)/eps
    return Jv

def solve_ik_position(pos_des, q0, l1, l2, l3, l4,
                      max_iter=200, tol=1e-4, alpha=0.6, max_step=np.deg2rad(5.0)):
    q = q0.copy()
    for _ in range(max_iter):
        _, p = fk_4dof(q, l1,l2,l3,l4)
        e = pos_des - p
        if np.linalg.norm(e) < tol:
            break
        Jv = numeric_Jv(q, l1,l2,l3,l4)
        dq = alpha * (np.linalg.pinv(Jv) @ e)
        dq = np.clip(dq, -max_step, max_step)
        q = wrap_to_pi(q + dq)
    return q

# --------- ROS wrapper: publish 5 joints (Joint_4 = 0, Joint_5 = q4) ----------
class IKPosition5JNode:
    def __init__(self):
        rospy.init_node("ik_position_5j_node")

        # Desired EE (m)
        self.pos_des = np.array([
            rospy.get_param("~xd", 0.2),
            rospy.get_param("~yd", 0.1),
            rospy.get_param("~zd", 0.4)
        ], dtype=float)

        # Robot parameters
        self.l1 = rospy.get_param("~l1", 0.0885)
        self.l2 = rospy.get_param("~l2", 0.140)
        self.l3 = rospy.get_param("~l3", 0.1329)
        self.l4 = rospy.get_param("~l4", 0.1105)

        # IK settings
        q0_deg = rospy.get_param("~q0_deg", [3,6,6,9])
        self.q0 = np.deg2rad(np.array(q0_deg, dtype=float))
        self.alpha    = rospy.get_param("~alpha", 0.6)
        self.tol      = rospy.get_param("~tol", 1e-4)
        self.max_iter = rospy.get_param("~max_iter", 200)
        self.max_step = np.deg2rad(rospy.get_param("~max_step_deg", 5.0))

        # Joint command topics (5 joints total)
        self.jtmap = rospy.get_param("~joint_topic_map",
            {'Joint_1':'/Joint_1/command',
             'Joint_2':'/Joint_2/command',
             'Joint_3':'/Joint_3/command',
             'Joint_4':'/Joint_4/command',
             'Joint_5':'/Joint_5/command'}
        )
        self.pubs = {j: rospy.Publisher(t, Float64, queue_size=10, latch=True) for j,t in self.jtmap.items()}

        # Also allow new goals on /ik/goal
        rospy.Subscriber("/ik/goal", Point, self.goal_cb, queue_size=1)

        # joint_states (so your FK node can subscribe too if you prefer)
        self.pub_js = rospy.Publisher("joint_states", JointState, queue_size=10, latch=True)

        # Solve once at startup
        self.solve_and_publish(self.pos_des)

    def goal_cb(self, msg: Point):
        self.pos_des = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.solve_and_publish(self.pos_des)

    def solve_and_publish(self, pos_des):
        q = solve_ik_position(pos_des, self.q0, self.l1,self.l2,self.l3,self.l4,
                              max_iter=self.max_iter, tol=self.tol,
                              alpha=self.alpha, max_step=self.max_step)

        # Map 4-DOF solution -> 5 joints: J1=q1, J2=q2, J3=q3, J4=0, J5=q4
        joint_positions = {
            'Joint_1': float(q[0]),
            'Joint_2': float(q[1]),
            'Joint_3': float(q[2]),
            'Joint_4': 0.0,
            'Joint_5': float(q[3]),
        }

        # Publish per-joint commands
        for j, pub in self.pubs.items():
            pub.publish(joint_positions.get(j, 0.0))

        # Publish /joint_states snapshot (optional but useful)
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = list(self.jtmap.keys())
        js.position = [joint_positions[n] for n in js.name]
        self.pub_js.publish(js)

        # Log
        qdeg = np.rad2deg([joint_positions['Joint_1'],
                           joint_positions['Joint_2'],
                           joint_positions['Joint_3'],
                           joint_positions['Joint_5']])
        rospy.loginfo("IK (deg): q1=%.3f q2=%.3f q3=%.3f q5=%.3f (J4 fixed 0)",
                      qdeg[0], qdeg[1], qdeg[2], qdeg[3])

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    IKPosition5JNode().spin()

