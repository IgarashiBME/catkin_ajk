#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# using GNSS UTM cordinate date

import rospy
import numpy as np
import time

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

easting_const = 368000   #[meter]
northing_const = 3955746 #[meter]

class gnss_dummy():
    def __init__(self):
        rospy.init_node('gnss_imu_sim_ajk_node')
        rospy.on_shutdown(self.shutdown)
        self.utm = Odometry()
        self.pub_utm = rospy.Publisher('/gnss_imu', Odometry, queue_size = 1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback, queue_size=1) # ROS callback function

    def callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "sim_ajk":
                self.x = msg.pose[i].position.x
                self.y = msg.pose[i].position.y
                self.z = msg.pose[i].position.z
                self.qx = msg.pose[i].orientation.x
                self.qy = msg.pose[i].orientation.y
                self.qz = msg.pose[i].orientation.z
                self.qw = msg.pose[i].orientation.w

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.x
                self.y
                self.z
            except AttributeError:
                continue
            self.utm.header.stamp = rospy.Time.now()
            self.utm.pose.pose.position.x = self.x + easting_const
            self.utm.pose.pose.position.y = self.y + northing_const
            self.utm.pose.pose.position.z = self.z

            self.utm.pose.pose.orientation.x = self.qx
            self.utm.pose.pose.orientation.y = self.qy
            self.utm.pose.pose.orientation.z = self.qz
            self.utm.pose.pose.orientation.w = self.qw
            self.pub_utm.publish(self.utm) 
            time.sleep(0.2)

    def shutdown(self):
        rospy.loginfo("sub_robot_node was terminated")

if __name__ == '__main__':
    g = gnss_dummy()
    g.loop()
