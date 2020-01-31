#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class look_ahead():
    def __init__(self):
        rospy.init_node('gnss_imu_fusion_node')
        rospy.on_shutdown(shutdown)

        # ROS callback
        rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size = 1)
        rospy.Subscriber('/gnss_odom', Odometry, self.odom_callback, queue_size = 1)

        # init valiables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gnss_q = np.empty(4)
        self.x = 0
        self.y = 0

    def imu_callback(self, msg):
        # quaternion
        self.imu_q[0] = imu_msg.orientation.x
        self.imu_q[1] = imu_msg.orientation.y
        self.imu_q[2] = imu_msg.orientation.z
        self.imu_q[3] = imu_msg.orientation.w

    def odom_callback(self, msg):
        # UTM coordinate
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # quaternion from yaw data of GNSS Moving-Base (roll, pitch are zero)
        self.gnss_q[0] = msg.pose.pose.orientation.x
        self.gnss_q[1] = msg.pose.pose.orientation.y
        self.gnss_q[2] = msg.pose.pose.orientation.z
        self.gnss_q[3] = msg.pose.pose.orientation.w

def utm(utm_msg):
    global utm_cordinate
    utm_coordinate[0] = utm_msg.pose.pose.position.x
    utm_coordinate[1] = utm_msg.pose.pose.position.y
    utm_coordinate[2] = utm_msg.pose.pose.position.z

def shutdown():
    rospy.loginfo("gnss_imu_node was terminated")

if __name__ == '__main__':
    s = slope_correction()
    s.loop()
