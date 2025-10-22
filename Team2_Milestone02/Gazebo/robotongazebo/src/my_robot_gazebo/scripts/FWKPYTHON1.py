#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math  # to convert degrees to radians

def deg_to_rad(deg):
    """Convert degrees to radians"""
    return deg * math.pi / 180.0

def main():
    # Initialize ROS node
    rospy.init_node('joint_publisher_node')

    # Create publishers for each joint
    pub_joint1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
    pub_joint2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
    pub_joint3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
    pub_joint4 = rospy.Publisher('/Joint_4/command', Float64, queue_size=10)
    pub_joint5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)

    # Wait a bit to ensure publishers are connected
    rospy.sleep(1)

    # 👉 Define angles in DEGREES here 👇
    angles_deg = [90, 90, 90, 0, 45]  # example values

    # Convert all angles to radians
    angles_rad = [deg_to_rad(a) for a in angles_deg]

    # Publish the joint angles
    pub_joint1.publish(angles_rad[0])
    pub_joint2.publish(angles_rad[1])
    pub_joint3.publish(angles_rad[2])
    pub_joint4.publish(angles_rad[3])
    pub_joint5.publish(angles_rad[4])

    rospy.loginfo(f"Published angles (deg): {angles_deg}")
    rospy.loginfo(f"Published angles (rad): {angles_rad}")

    # Keep node alive shortly
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
