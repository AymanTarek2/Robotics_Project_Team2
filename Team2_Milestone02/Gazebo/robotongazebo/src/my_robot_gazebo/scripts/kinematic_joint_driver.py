#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelConfiguration
import threading
import math
import time

def almost_equal(a, b, eps=1e-4):
    return abs(a - b) <= eps or (math.isfinite(a) and math.isfinite(b) and abs((a-b)/(abs(b)+1e-9)) < eps)

class KinematicJointDriver:
    def __init__(self):
        self.model_name  = rospy.get_param("~model_name", "my_robot")
        self.urdf_param  = rospy.get_param("~urdf_param", "robot_description")
        self.rate_hz     = rospy.get_param("~rate_hz", 30)
        self.epsilon     = rospy.get_param("~epsilon", 1e-4)   # change threshold to trigger an update
        self.publish_js  = rospy.get_param("~publish_joint_states", True)

        # Required: mapping of joint -> topic names
        # example param:
        #  joint_topic_map: {Joint_1: /Joint_1/command, Joint_2: /Joint_2/command, ...}
        joint_topic_map  = rospy.get_param("~joint_topic_map", {})
        if not joint_topic_map:
            rospy.logfatal("~joint_topic_map is empty. Example: {Joint_1: /Joint_1/command, Joint_2: /Joint_2/command, ...}")
            raise SystemExit

        self.joint_names = list(joint_topic_map.keys())
        self.joint_indices = {name: i for i, name in enumerate(self.joint_names)}
        self.values_lock = threading.Lock()
        self.values = [0.0]*len(self.joint_names)
        self.last_sent = [float('nan')]*len(self.joint_names)
        self.had_any_msg = False

        # Service
        rospy.wait_for_service("/gazebo/set_model_configuration")
        self.set_cfg = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        # Subscribers: one per joint
        for jname, topic in joint_topic_map.items():
            rospy.Subscriber(topic, Float64, self._make_cb(jname), queue_size=1)
            rospy.loginfo("Listening: %s  <-  %s", jname, topic)

        # Optional publisher for /joint_states (so TF updates)
        self.pub_js = rospy.Publisher("joint_states", JointState, queue_size=10) if self.publish_js else None
        self.js_msg = JointState()
        self.js_msg.name = self.joint_names
        rospy.loginfo("Kinematic driver ready for model '%s' (%d joints).", self.model_name, len(self.joint_names))

    def _make_cb(self, jname):
        idx = self.joint_indices[jname]
        def _cb(msg):
            val = float(msg.data)
            with self.values_lock:
                self.values[idx] = val
                self.had_any_msg = True
        return _cb

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            should_send = False
            with self.values_lock:
                if self.had_any_msg:
                    # Only send if any joint changed beyond epsilon
                    for i, v in enumerate(self.values):
                        prev = self.last_sent[i]
                        if not almost_equal(v, prev, self.epsilon):
                            should_send = True
                            break

                if should_send:
                    try:
                        resp = self.set_cfg(model_name=self.model_name,
                                            urdf_param_name=self.urdf_param,
                                            joint_names=self.joint_names,
                                            joint_positions=self.values)
                        if not resp.success:
                            rospy.logwarn_throttle(2.0, "set_model_configuration failed: %s", resp.status_message)
                        else:
                            self.last_sent = list(self.values)
                    except Exception as e:
                        rospy.logwarn_throttle(2.0, "set_model_configuration exception: %s", e)

            if self.pub_js and self.had_any_msg:
                self.js_msg.header.stamp = rospy.Time.now()
                # publish latest desired (what we just sent / will send)
                self.js_msg.position = list(self.values)
                self.js_msg.velocity = []
                self.js_msg.effort = []
                self.pub_js.publish(self.js_msg)

            r.sleep()

if __name__ == "__main__":
    rospy.init_node("kinematic_joint_driver")
    KinematicJointDriver().spin()

