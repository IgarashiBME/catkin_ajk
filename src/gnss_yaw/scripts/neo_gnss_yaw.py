#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# using GNSS UTM cordinate date

import rospy
import time
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

x = []
y = []
vel = np.array([])

opt_corr = 0.97 # threshold correlation coefficeint
opt_dist = 0.4 # threshold distance(unit:meter)

pub_yaw = rospy.Publisher('/gnss_yaw', Imu, queue_size = 10)
imu_msg = Imu()

linear_x = 0

def gnss_yaw(utm):
    global linear_x, straight_str, vel

    x.append(utm.pose.pose.position.x)    # read ROS /utm topic data
    y.append(utm.pose.pose.position.y)
    vel = np.append(vel, linear_x)

    if len(x) > 10:
        x.pop(0)
        y.pop(0)
        vel = np.delete(vel, 0)

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
        if dist > opt_dist and abs(corr[0,1]) > opt_corr > 0:
            if np.all(vel < 0) == True or np.all(vel > 0) == True:
                if yaw > np.pi:
                    yaw = yaw - 2*np.pi

                imu_msg.header.frame_id = "gnss_yaw"
                imu_msg.header.stamp = rospy.Time.now()
                q = quaternion_from_euler(0, 0, yaw)    # convert to quaternion (it is ROS function)
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
            
                pub_yaw.publish(imu_msg)
                e = euler_from_quaternion(q)
                #print vel
                #print np.all(vel < 0), np.all(vel > 0)
                print yaw, yaw/np.pi *180, e

def cmd_subs(msg):
    global linear_x
    linear_x = float(msg.linear.x)

def simulator_cmd(msg):
    global linear_x
    linear_x = float(msg.linear.x)

def shutdown():
    rospy.loginfo("gnss_yaw_node was terminated")

def listener():
    rospy.init_node('gnss_yaw_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('utm', Odometry, gnss_yaw) # ROS callback function
    rospy.Subscriber('cmd_vel', Twist, cmd_subs) # ROS callback function
    rospy.Subscriber('/sim_ajk/diff_drive_controller/cmd_vel', Twist, simulator_cmd) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
