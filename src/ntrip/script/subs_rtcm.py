#!/usr/bin/python
# coding:utf-8

import rospy
import binascii

# ROS message form
from std_msgs.msg import String

class ublox():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)

        # ROS subscriber
        rospy.Subscriber('/rtcm', String, self.rtcm_subs, queue_size=1)

    def loop(self):
        # searching for UBX-NAV-Class headers
        while not rospy.is_shutdown():
            x = 1
    def rtcm_subs(self, msg):
        print binascii.a2b_hex(msg.data)

    def shutdown(self):
        rospy.loginfo("ublox analyzer node was terminated") 

if __name__ == '__main__':
    rospy.init_node("ublox_analyzer_node")
    u = ublox()
    u.loop()
        
