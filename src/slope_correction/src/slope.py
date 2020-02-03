#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

ANTENNA_HEIGHT = 0.5

class slope_correction():
    def __init__(self):
        rospy.init_node('slope_correction_node')
        rospy.on_shutdown(self.shutdown)

        # ROS callback
        rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size = 1)
        rospy.Subscriber('/gnss_odom', Odometry, self.odom_callback, queue_size = 1)

        # init valiables
        self.imu_roll = 0
        self.pitch = 0
        self.x = 0
        self.y = 0

        rospy.spin()

    def imu_callback(self, msg):
        # quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        imu_e = euler_from_quaternion((qx, qy, qz, qw))
        self.imu_roll = imu_e[0]
        self.imu_pitch = imu_e[1]

    def odom_callback(self, msg):
        # UTM coordinate
        utm_x = msg.pose.pose.position.x
        utm_y = msg.pose.pose.position.y

        # quaternion from yaw data of GNSS Moving-Base (roll, pitch are zero)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        gnss_e = euler_from_quaternion((qx, qy, qz, qw))

        r = self.imu_roll
        p = self.imu_pitch
        y = gnss_e[2]

        # rotation matrix
        # roll
        Rx = np.array([[1,          0,         0],
                       [0,  np.cos(r), np.sin(r)],
                       [0, -np.sin(r), np.cos(r)]])

        # pitch
        Ry = np.array([[np.cos(p), 0, -np.sin(p)],
                       [0,         1,          0],
                       [np.sin(p), 0,  np.cos(p)]])   

        # yaw
        Rz = np.array([[ np.cos(y), np.sin(y), 0],
                       [-np.sin(y), np.cos(y), 0],
                       [         0,         0, 1]])

        R = Rz.dot(Ry).dot(Rz)

        # 3D-matrix from UTM and height of antenna
        A = np.array([[0],
                      [0],
                      [ANTENNA_HEIGHT]])

        B = R.dot(A)
        #print r/np.pi*180, p/np.pi*180
        print B

        corrected_x = utm_x - B[0]
        corrected_y = utm_y - B[1]

        print corrected_x, corrected_y

    def shutdown(self):
        rospy.loginfo("slope_correction node was terminated")

if __name__ == '__main__':
    s = slope_correction()
