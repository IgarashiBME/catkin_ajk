#! /usr/bin/env python

# pid control by euler angle

import rospy
import time
import numpy as np
import csv
import os

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

q = [0,0,0,0]    # quaternion
target_yaw = 0

i = 0
target_x = []
target_y = []
backward_flag = []

pre_yaw = 0
pre_time = 0

angular_const = 0.2    # m/s
twist = Twist()

kp = 0.5
ki = 0.1

def propotional(now_deviation):
    if now_deviation > 0:
        prop_angular_z = angular_const + (now_deviation/180 * kp)
    elif now_deviation < 0:
        prop_angular_z = -angular_const + (now_deviation/180 * kp)
    else:
        return 0

    if prop_angular_z > 0.55:
        prop_angular_z = 0.55
    elif prop_angular_z < -0.55:
        prop_angular_z = -0.55

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
    global pre_time, pre_yaw, i, backward_flag, target_x, target_y

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
    if backward_flag[i] == 1:                              # backward
        target_yaw = target_yaw + 180
        if target_yaw > 360:
            target_yaw = target_yaw - 360

    a = np.array([x, y])
    b = np.array([target_x[i], target_y[i]])    
    target_dist = np.linalg.norm(b-a)

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
    #print "distance=", target_dist
    #print "target_yaw=", target_yaw, "now_yaw=", now_yaw, "pre_yaw=", pre_yaw, 
    #print "deviation=", now_deviation
    #print "p=", prop, "i=", inte
    #print "waypoint_num=", i

    # vehicle control section
    if backward_flag[i] == 1:
        if target_dist < 0.2:
            i = i + 1
        elif abs(now_deviation) > 3:
            twist.linear.x = 0
            if twist.angular.z > 0 and twist.angular.z < 0.45:
                twist.angular.z = 0.45
            elif twist.angular.z < 0 and twist.angular.z > -0.45:
                twist.angular.z = -0.45
            pub.publish(twist)
        elif abs(now_deviation) < 3:
            twist.linear.x = -0.6
            twist.angular.z = 0
            pub.publish(twist)
    else:
        if target_dist < 0.2:
            i = i + 1
        elif abs(now_deviation) > 3:
            twist.linear.x = 0
            if twist.angular.z > 0 and twist.angular.z < 0.45:
                twist.angular.z = 0.45
            elif twist.angular.z < 0 and twist.angular.z > -0.45:
                twist.angular.z = -0.45
            pub.publish(twist)
        elif abs(now_deviation) < 3:
            twist.linear.x = 0.6
            twist.angular.z = 0
            pub.publish(twist)

    if i == len(target_x):
        i = 0
    #print twist.linear.x, twist.angular.z

    pre_yaw = now_yaw
    pre_time = now_time


def shutdown():
    rospy.loginfo("mower_control_node was terminated")

def listener():
    rospy.init_node('euler_control')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/gnss_imu', Odometry, euler) # ROS callback function, receive /odom mesage
    rospy.spin()

if __name__ == '__main__':
    csvdir = os.path.abspath(os.path.dirname(__file__))
    os.chdir(csvdir)
    f = open('route.csv', 'r')
    reader = csv.reader(f)
    line_count = 0
    row_count = 0

    for row in reader:
        for data in row:
            line_count += 1
            if line_count == 1:
                target_x.append(float(data))
            if line_count == 2:
                target_y.append(float(data))
            if line_count == 3:
                backward_flag.append(int(data))
                line_count = 0
        row_count += 1

    print target_x
    print target_y
    print backward_flag

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    listener()
