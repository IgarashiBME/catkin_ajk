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

        rospy.spin()

    def imu(self, msg):
        imu_e = euler_from_quaternion((msg.orientation.x, 
                                       msg.orientation.y, 
                                       msg.orientation.z, 
                                       msg.orientation.w))
        # 0~2 pi radian
        if imu_e[2] < 0:
            imu_yaw = imu_e[2] + 2*np.pi
        else:
            imu_yaw = imu_e[2]

        try:
            self.imu_diff = imu_yaw - self.pre_imu_yaw
        except AttributeError:
            pass

        self.pre_imu_yaw = imu_yaw

        # add the diffrence of imu_yaw to gnss_yaw
        try:
            self.fusion_yaw = self.fusion_yaw + self.imu_diff
            print self.fusion_yaw/np.pi*180
            if self.fusion_yaw < -np.pi:
                self.fusion_yaw = self.fusion_yaw + np.pi *2
            if self.fusion_yaw > np.pi:
                self.fusion_yaw = self.fusion_yaw - np.pi*2
            fusion_q = quaternion_from_euler(0, 0, self.fusion_yaw)

            self.gnss_imu.pose.pose.position.x = self.utm_x
            self.gnss_imu.pose.pose.position.y = self.utm_y
            self.gnss_imu.pose.pose.position.z = self.utm_z
            self.gnss_imu.pose.pose.orientation.x = fusion_q[0]
            self.gnss_imu.pose.pose.orientation.y = fusion_q[1]
            self.gnss_imu.pose.pose.orientation.z = fusion_q[2]
            self.gnss_imu.pose.pose.orientation.w = fusion_q[3]

            self.pub.publish(self.gnss_imu)
        except AttributeError:
            pass

    def gnss_yaw(self, msg):
        gnss_e = euler_from_quaternion((msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z,
                                        msg.orientation.w))
        # 0~2 pi radian
        if gnss_e[2] < 0:
            self.fusion_yaw = gnss_e[2] + 2*np.pi
        else:
            self.fusion_yaw = gnss_e[2]

    def utm(self, msg):
        self.utm_x = msg.pose.pose.position.x
        self.utm_y = msg.pose.pose.position.y
        self.utm_z = msg.pose.pose.position.z

    def shutdown(self):
        rospy.loginfo("fusion_gnss_imu_node was terminated")

if __name__ == '__main__':
    f = fusion()
