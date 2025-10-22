#!/usr/bin/env python3
import math, ast, rospy, numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion

# ===================== YOUR ROBOT SETTINGS =====================
# End-effector link name exactly as it appears in /gazebo/link_states
EE_LINK_NAME = "my_robot::EE_frame"    # <-- change if your EE link has a different name

# Joint command topics (first 4 are used by this script to match your MATLAB model)
CMD_TOPICS = [
    "/Joint_1/command",
    "/Joint_2/command",
    "/Joint_3/command",
    "/Joint_4/command",
    # "/Joint_5/command",  # optional: uncomment if you want to also command J5 (held at 0 by default)
]
# ===============================================================

# ---------- MATLAB-like helpers ----------
def cosd(x): return math.cos(math.radians(x))
def sind(x): return math.sin(math.radians(x))

# ---------- Your link lengths (meters) ----------
l1 = 0.0885
l2 = 0.140
l3 = 0.1329
l4 = 0.1105

def fk_matlab_style(q1, q2, q3, q4):
    """Reproduces your MATLAB FK exactly (degrees in, X/Y/Z out in meters)."""
    T1 = np.array([
        [ cosd(q1),        -sind(q1)*cosd(90),   sind(q1)*sind(90),  0.0 ],
        [ sind(q1),         cosd(q1)*cosd(90),  -cosd(q1)*sind(90),  0.0 ],
        [ 0.0,              sind(90),            cosd(90),           l1  ],
        [ 0.0,              0.0,                 0.0,                1.0 ],
    ], dtype=float)

    T2 = np.array([
        [ cosd(q2+90),      -sind(q2+90)*cosd(180),  sind(q2+90)*sind(180),  l2*cosd(q2+90) ],
        [ sind(q2+90),       cosd(q2+90)*cosd(180), -cosd(q2+90)*sind(180),  l2*sind(q2+90) ],
        [ 0.0,               sind(180),              cosd(180),              0.0           ],
        [ 0.0,               0.0,                    0.0,                   1.0           ],
    ], dtype=float)

    T3 = np.array([
        [ cosd(q3),         -sind(q3)*cosd(180),  sind(q3)*sind(180),  l3*cosd(q3) ],
        [ sind(q3),          cosd(q3)*cosd(180), -cosd(q3)*sind(180),  l3*sind(q3) ],
        [ 0.0,               sind(180),           cosd(180),           0.0         ],
        [ 0.0,               0.0,                 0.0,                 1.0         ],
    ], dtype=float)

    T4 = np.array([
        [ cosd(q4),         -sind(q4)*cosd(0),   sind(q4)*sind(0),   l4*cosd(q4) ],
        [ sind(q4),          cosd(q4)*cosd(0),  -cosd(q4)*sind(0),   l4*sind(q4) ],
        [ 0.0,               sind(0),           cosd(0),            0          ],
        [ 0.0,               0.0,                0.0,                1.0         ],
    ], dtype=float)

    T = T1 @ T2 @ T3 @ T4
    return T[0,3], T[1,3], T[2,3]   # x, y, z

class CompareFK:
    def __init__(self):
        rospy.init_node("compare_fk_b", anonymous=True)

        # Allow overrides from roslaunch/command line
        self.ee_link_name = rospy.get_param("~ee_link_name", EE_LINK_NAME)
        self.cmd_topics   = rospy.get_param("~cmd_topics", CMD_TOPICS)

        # angles in degrees [q1,q2,q3,q4] (same angles you used in MATLAB)
        q_deg_param = rospy.get_param("~q_deg", "[0, 90,90, 0]")
        self.q_deg = ast.literal_eval(q_deg_param) if isinstance(q_deg_param, str) else q_deg_param
        if len(self.q_deg) != 4:
            rospy.logfatal("~q_deg must be a list of 4 angles (degrees). Got: %r", self.q_deg)
            raise SystemExit

        self.pubs = [rospy.Publisher(t, Float64, queue_size=10) for t in self.cmd_topics[:4]]
        self.ee_pose = None
        rospy.Subscriber("/gazebo/link_states", LinkStates, self._on_links, queue_size=1)

    def _on_links(self, msg: LinkStates):
        try:
            i = msg.name.index(self.ee_link_name)
            self.ee_pose = msg.pose[i]
        except ValueError:
            pass

    def send_commands(self):
        """Publish the 4 angles (converted to radians) several times so the driver picks them up."""
        q_rad = [math.radians(v) for v in self.q_deg]
        msgs = [Float64(data=v) for v in q_rad]
        rospy.sleep(0.15)
        for _ in range(6):
            for p, m in zip(self.pubs, msgs):
                p.publish(m)
            rospy.sleep(0.03)

    def sample_gazebo_xyz(self, samples=10, hz=30):
        """Read EE pose a few times and average (smoother)."""
        if not self.ee_pose:
            rospy.sleep(0.2)
        rate = rospy.Rate(hz)
        xs, ys, zs = [], [], []
        for _ in range(samples):
            if self.ee_pose:
                p = self.ee_pose.position
                xs.append(p.x); ys.append(p.y); zs.append(p.z)
            rate.sleep()
        if xs:
            return sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)
        return None

    def run(self):
        # 1) Command Gazebo (first 4 joints)
        self.send_commands()

        # 2) FK (your MATLAB calculation)
        x_fk, y_fk, z_fk = fk_matlab_style(*self.q_deg)

        # 3) Read Gazebo EE pose (average a bit)
        gazebo_xyz = self.sample_gazebo_xyz()
        if gazebo_xyz is None:
            rospy.logwarn("Could not read EE pose from Gazebo.")
            return
        x_g, y_g, z_g = gazebo_xyz

        # 4) Print comparison
        print("\n================= PART (b) RESULT =================")
        print(f"Joint angles (deg) : {self.q_deg}")
        print(f"Gazebo   EE (m)    : x={x_g:.4f}  y={y_g:.4f}  z={z_g:.4f}")
        print(f"FK (MATLAB) EE (m) : x={x_fk:.4f}  y={y_fk:.4f}  z={z_fk:.4f}")
        print(f"Position error (m) : dx={x_g-x_fk:.4f}  dy={y_g-y_fk:.4f}  dz={z_g-z_fk:.4f}")
        print("===================================================\n")

if __name__ == "__main__":
    CompareFK().run()

