#!/usr/bin/env python3
import math, ast
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from rospy.exceptions import ROSTimeMovedBackwardsException
from tf.transformations import euler_from_quaternion

def deg2rad_list(q_deg):
    return [math.radians(float(v)) for v in q_deg]

class PartAController:
    def __init__(self):
        rospy.init_node("part_a_controller", anonymous=True)

        # ---- Params (edit in launch or pass on command line) ----
        self.joint_topics = rospy.get_param("~joint_topics", [
            "/Joint_1/command",
            "/Joint_2/command",
            "/Joint_3/command",
            "/Joint_4/command",
            "/Joint_5/command",
        ])
        # End-effector link name in Gazebo (as shown in /gazebo/link_states)
        self.ee_link_name = rospy.get_param("~ee_link_name", "my_robot::EE_frame")

        # Desired joint angles in *degrees* (5 values). Ex: "[10, -5, 20, 0, 15]"
        q_deg_param = rospy.get_param("~q_deg", "[0, 90, 0, 0, 0]")
        if isinstance(q_deg_param, str):
            q_deg = ast.literal_eval(q_deg_param)
        else:
            q_deg = q_deg_param
        if len(q_deg) != 5:
            rospy.logfatal("~q_deg must have exactly 5 numbers (degrees). Got: %r", q_deg)
            raise SystemExit
        self.q_rad = deg2rad_list(q_deg)

        # ---- Publishers & Subscriber ----
        self.pubs = [rospy.Publisher(t, Float64, queue_size=10) for t in self.joint_topics]
        self.ee_pose = None
        rospy.Subscriber("/gazebo/link_states", LinkStates, self._on_link_states, queue_size=1)

        rospy.loginfo("Part (a): commanding joints (deg) -> %s", q_deg)
        rospy.loginfo("EE link: %s", self.ee_link_name)

    # Grab EE pose from Gazebo
    def _on_link_states(self, msg: LinkStates):
        try:
            idx = msg.name.index(self.ee_link_name)
            self.ee_pose = msg.pose[idx]
        except ValueError:
            # EE link not in the message yet; ignore
            pass

    # Publish desired angles a few times (so the driver certainly receives them)
    def send_joint_targets(self, repeats=5, sleep_sec=0.03):
        rospy.sleep(0.15)  # let pubs connect
        msg = [Float64(data=v) for v in self.q_rad]
        for _ in range(repeats):
            for p, m in zip(self.pubs, msg):
                p.publish(m)
            rospy.sleep(sleep_sec)
        rospy.loginfo("Joint commands sent.")

    # Print EE pose for a few seconds
    def print_ee_pose(self, hz=10, seconds=3.0):
        rate = rospy.Rate(hz)
        steps = int(hz * seconds)
        for _ in range(steps):
            try:
                if self.ee_pose:
                    p = self.ee_pose.position
                    q = self.ee_pose.orientation
                    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                    print(f"EE pose: x={p.x:.4f}  y={p.y:.4f}  z={p.z:.4f}  "
                          f"roll={roll:.4f}  pitch={pitch:.4f}  yaw={yaw:.4f}")
                rate.sleep()
            except ROSTimeMovedBackwardsException:
                rospy.logwarn("Sim time reset detected; continuing.")
                continue

if __name__ == "__main__":
    node = PartAController()
    node.send_joint_targets()
    node.print_ee_pose()

