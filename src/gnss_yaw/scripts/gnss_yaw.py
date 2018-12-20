#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# using GNSS UTM cordinate date

import rospy
import time
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

x = []
y = []

opt_corr = 0.95 # threshold correlation coefficeint
opt_dist = 0.4 # threshold distance(unit:meter)

pub_yaw = rospy.Publisher('/gnss_yaw', Imu, queue_size = 10)
imu_msg = Imu()

linear_x = 0
straight_str = 0

def gnss_yaw(utm):
    global linear_x, straight_str

    x.append(utm.pose.pose.position.x)    # read ROS /utm topic data
    y.append(utm.pose.pose.position.y)

    if len(x) > 10:
        x.pop(0)
        y.pop(0)

    if len(x) > 9:
        # calculate yaw from trajectory
        yaw = np.arctan2(y[9]-y[0], x[9]-x[0])

        # when vehicle move backward, calculate yaw have to turn 3.1415... radian (180degree)
        if linear_x < 0 and yaw < 0:
            yaw = yaw + np.pi
        elif linear_x < 0 and yaw > 0:
            yaw = yaw - np.pi        

        # linearity check of trajectory
        corr = np.corrcoef(x, y)    # calculate correlation coefficeint
        dist = np.linalg.norm((y[9]-y[0]-(x[9]-x[0])))    # calculate Euclidean distance
        dist_latter = np.linalg.norm((y[9]-y[8]-(x[9]-x[8])))    # calculate Euclidean distance

        # If you get optimal distance and correlation coefficeint, 
        # publish ROS messages with GNSS yaw(quaternion)
        if dist > opt_dist and abs(corr[0,1]) > opt_corr > 0 and straight_str == "straight":
            imu_msg.header.frame_id = "gnss_yaw"
            imu_msg.header.stamp = rospy.Time.now()
            q = quaternion_from_euler(0, 0, yaw)    # convert to quaternion (it is ROS function)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            
            pub_yaw.publish(imu_msg)
            e = euler_from_quaternion(q)
            print yaw, yaw/np.pi *180, e

def cmd_subs(msg):
    global linear_x
    linear_x = msg.linear.x

def simulator_cmd(msg):
    global linear_x
    linear_x = msg.linear.x

def straight_subs(msg):
    global straight_str
    straight_str = msg.data

def shutdown():
    rospy.loginfo("gnss_yaw_node was terminated")

def listener():
    rospy.init_node('gnss_yaw_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('utm', Odometry, gnss_yaw) # ROS callback function
    rospy.Subscriber('cmd_vel', Twist, cmd_subs) # ROS callback function
    rospy.Subscriber('straight_str', String, straight_subs) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
