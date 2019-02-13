#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ubx_analyzer.msg import UTMHP

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class fusion():
    def __init__(self):
        rospy.init_node('moving_base_and_imu_fusion_node')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function
        rospy.Subscriber('/imu/data', Imu, self.imu)
        rospy.Subscriber('/moving_base', Imu, self.moving_base)
        rospy.Subscriber('/utm_hp', UTMHP, self.utm_hp)
        # ROS publish function
        self.pub = rospy.Publisher('/gnss_imu_odom', Odometry, queue_size = 1)
        self.gnss_imu_odom = Odometry()

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

        # calculate diffrence of imu yaw with current and previous sequence
        try:
            self.imu_diff = imu_yaw - self.pre_imu_yaw
        except AttributeError:
            pass
        self.pre_imu_yaw = imu_yaw

        # add the diffrence of imu_yaw to gnss_yaw
        try:
            self.fusion_yaw = self.fusion_yaw + self.imu_diff

            if self.fusion_yaw < -np.pi:
                self.fusion_yaw = self.fusion_yaw + np.pi *2
            if self.fusion_yaw > np.pi:
                self.fusion_yaw = self.fusion_yaw - np.pi*2
            fusion_q = quaternion_from_euler(0, 0, self.fusion_yaw)
            print self.fusion_yaw/np.pi*180

            self.gnss_imu_odom.pose.pose.position.x = self.utm_x
            self.gnss_imu_odom.pose.pose.position.y = self.utm_y
            self.gnss_imu_odom.pose.pose.position.z = self.utm_z
            self.gnss_imu_odom.pose.pose.orientation.x = fusion_q[0]
            self.gnss_imu_odom.pose.pose.orientation.y = fusion_q[1]
            self.gnss_imu_odom.pose.pose.orientation.z = fusion_q[2]
            self.gnss_imu_odom.pose.pose.orientation.w = fusion_q[3]

            self.pub.publish(self.gnss_imu_odom)
        except AttributeError:
            pass

    def moving_base(self, msg):
        gnss_e = euler_from_quaternion((msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z,
                                        msg.orientation.w))
        # 0~2 pi radian
        if gnss_e[2] < 0:
            self.fusion_yaw = gnss_e[2] + 2*np.pi
        else:
            self.fusion_yaw = gnss_e[2]

    def utm_hp(self, msg):
        self.utm_x = msg.utm_easting
        self.utm_y = msg.utm_northing
        self.utm_z = msg.heightHp

    def shutdown(self):
        rospy.loginfo("fusion_imu_moving_base_node was terminated")

if __name__ == '__main__':
    f = fusion()
