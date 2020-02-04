#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class fake():
    def __init__(self):
        rospy.init_node('fake_imu_odom_node')
        rospy.on_shutdown(self.shutdown)

        # ROS publisher
        self.pub_odom = rospy.Publisher('/gnss_odom', Odometry, queue_size = 1)
        self.odom = Odometry()

        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size = 1)
        self.imu = Imu()

    def shutdown(self):
        rospy.loginfo("slope_correction node was terminated")

    def loop(self):
        rr = rospy.Rate(10)
        while not rospy.is_shutdown():
            odom_q = quaternion_from_euler(0.0, 0.0, 0.0)
            self.odom.header.stamp = rospy.Time.now()
            self.odom.pose.pose.position.x = 1.0
            self.odom.pose.pose.position.y = 1.0
            self.odom.pose.pose.position.z = 1.0
            self.odom.pose.pose.orientation.x = odom_q[0]
            self.odom.pose.pose.orientation.y = odom_q[1]
            self.odom.pose.pose.orientation.z = odom_q[2]
            self.odom.pose.pose.orientation.w = odom_q[3]
            self.pub_odom.publish(self.odom)

            imu_q = quaternion_from_euler(30.0/180.0*np.pi, 0.0, 0.0)
            self.imu.header.stamp = rospy.Time.now()
            self.imu.orientation.x = imu_q[0]
            self.imu.orientation.y = imu_q[1]
            self.imu.orientation.z = imu_q[2]
            self.imu.orientation.w = imu_q[3]
            self.pub_imu.publish(self.imu)

            rr.sleep()

if __name__ == '__main__':
    f = fake()
    f.loop()
    
