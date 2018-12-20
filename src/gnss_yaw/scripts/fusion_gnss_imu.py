#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class fusion():
    def __init__(self):
        rospy.init_node('gnss_imu_fusion_node')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function
        rospy.Subscriber('/imu/data', Imu, self.imu)
        rospy.Subscriber('/gnss_yaw', Imu, self.gnss_yaw) # vehicle's heading from arctan2 with UTM coordinate
        rospy.Subscriber('/utm', Odometry, self.utm)
        # ROS publish function
        self.pub = rospy.Publisher('/gnss_imu', Odometry, queue_size = 10)
        self.gnss_imu = Odometry()

        self.pre_seq = 0
        rospy.spin()

    def imu(self, msg):
        imu_e = euler_from_quaternion((msg.orientation.x, 
                                       msg.orientation.y, 
                                       msg.orientation.z, 
                                       msg.orientation.w))
        # 0~2 pi radian
        if imu_e[2] < 0:
            self.imu_yaw = imu_e[2] + 2*np.pi
        else:
            self.imu_yaw = imu_e[2]

    def gnss_yaw(self, msg):
        self.gnss_yaw_seq = msg.header.seq
        gnss_e = euler_from_quaternion((msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z,
                                        msg.orientation.w))
        # 0~2 pi radian
        if gnss_e[2] < 0:
            self.gnss_yaw = gnss_e[2] + 2*np.pi
        else:
            self.gnss_yaw = gnss_e[2]

    def utm(self, msg):
        self.utm_x = msg.pose.pose.position.x
        self.utm_y = msg.pose.pose.position.y
        self.utm_z = msg.pose.pose.position.z

    def loop():
        while not rospy.is_shutdown():
            try:
                self.gnss_yaw
                self.gnss_yaw_seq
            except AttributeError:
                continue

            if pre_seq != self.gnss_yaw_seq
                fusion_yaw = self.gnss_yaw

    def shutdown(self):
        rospy.loginfo("fusion_gnss_imu_node was terminated")

if __name__ == '__main__':
    f = fusion()
