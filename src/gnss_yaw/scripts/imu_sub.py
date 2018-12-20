#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/gnss_imu', Odometry, queue_size = 10)

def imu(imu_msg):
    imu_e = euler_from_quaternion((imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w))
    imu_yaw = imu_e[2]/np.pi *180
    print "imu_", imu_yaw

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
    rospy.Subscriber('/imu/data', Imu, imu) # ROS callback function
    rospy.Subscriber('/gnss_imu', Odometry, gnss_imu) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
