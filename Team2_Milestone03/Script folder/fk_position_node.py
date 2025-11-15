#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# ---------- Forward Kinematics (radians) ----------
def fk_4dof_radians(q, l1, l2, l3, l4):
    """Return T (4x4) and p (3,) for the 4-DOF chain. q = [q1,q2,q3,q4] in radians."""
    q1, q2, q3, q4 = q
    pi = np.pi; c = np.cos; s = np.sin

    T1 = np.array([[ c(q1), -s(q1)*c(pi/2),  s(q1)*s(pi/2),  0.0],
                   [ s(q1),  c(q1)*c(pi/2), -c(q1)*s(pi/2),  0.0],
                   [   0.0,          s(pi/2),       c(pi/2),  l1 ],
                   [   0.0,              0.0,          0.0,  1.0]])

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
    return T, T[:3, 3]

class FKPositionNode:
    def __init__(self):
        rospy.init_node("fk_position_node")

        # Link lengths (same defaults you used)
        self.l1 = rospy.get_param("~l1", 0.0885)
        self.l2 = rospy.get_param("~l2", 0.140)
        self.l3 = rospy.get_param("~l3", 0.1329)
        self.l4 = rospy.get_param("~l4", 0.1105)

        # Names must match your /joint_states .name ordering (from your IK node)
        # Convention: J1=q1, J2=q2, J3=q3, J4 fixed=0, J5=q4
        # We'll map q = [Joint_1, Joint_2, Joint_3, Joint_5]
        self.joint_names = rospy.get_param("~joint_names",
                                           ["Joint_1", "Joint_2", "Joint_3", "Joint_5"])

        # Topic to listen to (your IK node already publishes a latched /joint_states)
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")

        # Publisher for EE position
        self.pub_point = rospy.Publisher("/fk/ee", Point, queue_size=10, latch=True)

        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_state_cb, queue_size=10)

        rospy.loginfo("[fk_position_node] Ready. Waiting for /joint_states...")

    def joint_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))

        # Extract q1,q2,q3,q4 (q4 comes from Joint_5)
        missing = [jn for jn in self.joint_names if jn not in name_to_pos]
        if missing:
            # If message doesn't carry all joints we need, skip quietly
            return

        q1 = float(name_to_pos[self.joint_names[0]])
        q2 = float(name_to_pos[self.joint_names[1]])
        q3 = float(name_to_pos[self.joint_names[2]])
        q4 = float(name_to_pos[self.joint_names[3]])  # this is Joint_5 in your IK node

        q = np.array([q1, q2, q3, q4], dtype=float)  # radians

        # Compute FK
        _, p = fk_4dof_radians(q, self.l1, self.l2, self.l3, self.l4)

        # Publish and log
        pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        self.pub_point.publish(pt)
        rospy.loginfo_throttle(0.5, "FK EE pos: x=%.4f  y=%.4f  z=%.4f", pt.x, pt.y, pt.z)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    FKPositionNode().spin()

