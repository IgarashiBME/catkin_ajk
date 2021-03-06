#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/gnss_imu', Odometry, queue_size = 10)

def imu(msg):
    imu_e = euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))

    r = imu_e[0]/np.pi *180
    p = imu_e[1]/np.pi *180
    y = imu_e[2]/np.pi *180
    print "roll :", r
    print "pitch:", p
    print "yaw  :", y, "\n"

def gnss_imu(msg):
    gnss_imu_e = euler_from_quaternion((msg.pose.pose.orientation.x, 
                                        msg.pose.pose.orientation.y, 
                                        msg.pose.pose.orientation.z, 
                                        msg.pose.pose.orientation.w))
    yaw = gnss_imu_e[2]/np.pi *180
    print "gnss", yaw

def shutdown():
    rospy.loginfo("imu_data_sub_node was terminated")

def listener():
    rospy.init_node('imu_data_sub_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/mad_imu/data', Imu, imu) # ROS callback function
    rospy.Subscriber('/gnss_imu', Odometry, gnss_imu) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
