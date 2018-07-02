#! /usr/bin/env python

# pid control by euler angle

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

q = [0,0,0,0]    # quaternion
target_yaw = 0

i = 0
target_x = [ 388001.86,  388001.00,  388001.00,  388001.00]
target_y = [3953233.93, 3953233.88, 3953235.78, 3953235.78]
pre_yaw = 0
pre_time = 0

angular_const = 0.1    # m/s
twist = Twist()

kp = 0.4
ki = 0.1

def propotional(now_deviation):
    if now_deviation > 0:
        prop_angular_z = angular_const + (now_deviation/180 * kp)
    elif now_deviation < 0:
        prop_angular_z = -angular_const + (now_deviation/180 * kp)
    else:
        return 0
    return prop_angular_z

def integral(pre_time, pre_yaw, now_time, now_deviation):
    if pre_yaw == 0:
        return 0

    # choose minimam deviation angle(pre)
    deviation_a = target_yaw - pre_yaw
    deviation_b = target_yaw - (pre_yaw + 360)
    deviation_c = (target_yaw + 360) - pre_yaw

    if abs(deviation_a) <= abs(deviation_b) and abs(deviation_a) <= abs(deviation_c): 
        pre_deviation = deviation_a
    elif abs(deviation_b) <= abs(deviation_a) and abs(deviation_b) <= abs(deviation_c):
        pre_deviation = deviation_b
    elif abs(deviation_c) <= abs(deviation_a) and abs(deviation_c) <= abs(deviation_b):
        pre_deviation = deviation_c    
    
    return (pre_deviation/180 + now_deviation/180) * (now_time - pre_time) /2 *ki
    
def euler(odom):
    global pre_time, pre_yaw, i

    now_time = odom.header.stamp.secs + odom.header.stamp.nsecs/1000000000.00

    # vehicle's position data in /odom (odometry of ROS message)
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y

    # vehicle's quaternion data in /odom (odometry of ROS message)
    q[0] = odom.pose.pose.orientation.x
    q[1] = odom.pose.pose.orientation.y
    q[2] = odom.pose.pose.orientation.z
    q[3] = odom.pose.pose.orientation.w

    e = euler_from_quaternion((q[0],q[1],q[2],q[3]))    # you get euler angles from quaternion
    now_yaw = e[2]/np.pi * 180
    if now_yaw < 0:    # yaw angle, 0~360 degree
        now_yaw = now_yaw + 360

    # calculate target_yaw and distance
    target_yaw = np.arctan2(target_y[i] - y, target_x[i] - x)/np.pi * 180
    if target_yaw < 0:
        target_yaw = target_yaw + 360

    a = np.array([x, y])
    b = np.array([target_x[i], target_y[i]])    
    target_dist = np.linalg.norm(b-a)

    print target_dist

    # choose minimam deviation angle(now)            
    deviation_a = target_yaw - now_yaw
    deviation_b = target_yaw - (now_yaw + 360)
    deviation_c = (target_yaw + 360) - now_yaw 

    if abs(deviation_a) <= abs(deviation_b) and abs(deviation_a) <= abs(deviation_c): 
        now_deviation = deviation_a
    elif abs(deviation_b) <= abs(deviation_a) and abs(deviation_b) <= abs(deviation_c):
        now_deviation = deviation_b
    elif abs(deviation_c) <= abs(deviation_a) and abs(deviation_c) <= abs(deviation_b):
        now_deviation = deviation_c

    prop = propotional(now_deviation)
    inte = integral(pre_time, pre_yaw, now_time, now_deviation)

    twist.angular.z = prop + inte

    #print now_time
    print target_yaw, now_yaw, pre_yaw, now_deviation
    print "prop=", prop
    print "inte=", inte
    print i

    if i == 3:
        i = 0

    if target_dist < 0.2:
        i = i + 1
    elif abs(now_deviation) > 15:
        twist.linear.x = 0
        pub.publish(twist)
    elif abs(now_deviation) < 15:
        twist.linear.x = 0.15
        pub.publish(twist)
    elif abs(now_deviation) < 20:
        twist.linear.x = 0.1
        pub.publish(twist)

    pre_yaw = now_yaw
    pre_time = now_time

    
def shutdown():
    rospy.loginfo("euler_control_node was terminated")

def listener():
    rospy.init_node('euler_control')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/gnss_imu', Odometry, euler) # ROS callback function, receive /odom mesage
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    listener()
