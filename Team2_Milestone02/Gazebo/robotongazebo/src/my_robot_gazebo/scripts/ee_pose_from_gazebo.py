#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node("ee_pose_from_gazebo")
    link_name = rospy.get_param("~link_name", "my_robot::Gripper")
    ref_frame = rospy.get_param("~reference_frame", "world")
    topic     = rospy.get_param("~topic", "end_effector_pose")
    rate_hz   = rospy.get_param("~rate", 30)

    rospy.wait_for_service("/gazebo/get_link_state")
    get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    pub = rospy.Publisher(topic, PoseStamped, queue_size=10)

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        try:
            resp = get_link(link_name, ref_frame)
            if resp.success:
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = ref_frame
                msg.pose = resp.link_state.pose
                pub.publish(msg)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "get_link_state failed: %s", e)
        r.sleep()

if __name__ == "__main__":
    main()

