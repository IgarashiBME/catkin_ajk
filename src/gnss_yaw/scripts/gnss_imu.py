#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/gnss_imu', Odometry, queue_size = 10)
gnss_imu = Odometry()

gnss_yaw_data = [0, 0, 0, 0, 0]     #gnss yaw quaternion
q = [0,0,0,0]
pre_gnss_seq = 0
gnss_euler_yaw = 0
utm_coordinate = [0,0,0]

fusion_flag = 0

def imu(imu_msg):
    global gnss_yaw_data
    global gnss_euler_yaw
    global pre_gnss_seq
    global pre_imu_yaw
    global fusion_flag
    global fusion_yaw
    global utm_coordinate

    imu_e = euler_from_quaternion((imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w))
    imu_yaw = imu_e[2]
    # gnss_yaw is absolute orientation. If can't get gnss_yaw
    if pre_gnss_seq != gnss_yaw_data[0]:    # when gnss_yaw is new, then gnss_yaw uwagaki
        gnss_imu.pose.pose.orientation.x = gnss_yaw_data[1]
        gnss_imu.pose.pose.orientation.y = gnss_yaw_data[2]
        gnss_imu.pose.pose.orientation.z = gnss_yaw_data[3]
        gnss_imu.pose.pose.orientation.w = gnss_yaw_data[4]

        fusion_flag = 0        

    elif pre_gnss_seq == gnss_yaw_data[0] and fusion_flag == 1:
        imu_e = euler_from_quaternion((imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w))
        imu_yaw = imu_e[2]
        if imu_yaw < 0:
            imu_yaw = imu_yaw + np.pi *2

        fusion_yaw = gnss_euler_yaw + imu_yaw - pre_imu_yaw
        if fusion_yaw < -np.pi:
            fusion_yaw = fusion_yaw + np.pi *2
        if fusion_yaw > np.pi:
            fusion_yaw = fusion_yaw - np.pi*2
        
        fusion_q = quaternion_from_euler(0, 0, fusion_yaw)
        
        gnss_imu.pose.pose.orientation.x = fusion_q[0]
        gnss_imu.pose.pose.orientation.y = fusion_q[1]
        gnss_imu.pose.pose.orientation.z = fusion_q[2]
        gnss_imu.pose.pose.orientation.w = fusion_q[3]

    elif pre_gnss_seq == gnss_yaw_data[0] and fusion_flag == 0:
        q[0] = imu_msg.orientation.x
        q[1] = imu_msg.orientation.y
        q[2] = imu_msg.orientation.z
        q[3] = imu_msg.orientation.w

        e = euler_from_quaternion((q[0],q[1],q[2],q[3]))
        pre_imu_yaw = e[2]
        if pre_imu_yaw < 0:
            pre_imu_yaw = pre_imu_yaw + np.pi *2    # 0 ~ 6.28 radian range
        fusion_flag = 1

    gnss_imu.pose.pose.position.x = utm_coordinate[0]
    gnss_imu.pose.pose.position.y = utm_coordinate[1]
    gnss_imu.pose.pose.position.z = utm_coordinate[2]

    print "fusion:", fusion_yaw/np.pi *180, "gnss_yaw:", gnss_euler_yaw/np.pi * 180, "imu:",imu_yaw/np.pi * 180
    pre_gnss_seq = gnss_yaw_data[0]

    pub.publish(gnss_imu)

def gnss_yaw(gnss_yaw_msg):
    global gnss_yaw_data    # use gnss_quaternion in other functions
    global gnss_euler_yaw

    gnss_yaw_data[0] = gnss_yaw_msg.header.seq
    gnss_yaw_data[1] = gnss_yaw_msg.orientation.x
    gnss_yaw_data[2] = gnss_yaw_msg.orientation.y
    gnss_yaw_data[3] = gnss_yaw_msg.orientation.z
    gnss_yaw_data[4] = gnss_yaw_msg.orientation.w

    gnss_e = euler_from_quaternion((gnss_yaw_data[1],gnss_yaw_data[2],gnss_yaw_data[3],gnss_yaw_data[4]))
    gnss_euler_yaw = gnss_e[2]

    # print section
    #if gnss_euler_yaw < 0:
    #    gnss_euler_yaw = gnss_euler_yaw + np.pi *2

    print "gnss_yaw", gnss_euler_yaw, gnss_euler_yaw/np.pi *180  

def utm(utm_msg):
    global utm_cordinate
    utm_coordinate[0] = utm_msg.pose.pose.position.x
    utm_coordinate[1] = utm_msg.pose.pose.position.y
    utm_coordinate[2] = utm_msg.pose.pose.position.z

def shutdown():
    rospy.loginfo("gnss_imu_node was terminated")

def listener():
    rospy.init_node('gnss_imu_fusion_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/imu/data', Imu, imu) # ROS callback function
    rospy.Subscriber('/gnss_yaw', Imu, gnss_yaw) # ROS callback function
    rospy.Subscriber('/utm', Odometry, utm) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
