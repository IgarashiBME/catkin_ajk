#! /usr/bin/env python

import rospy
import numpy as np
import csv
import os
import time
import sys

import load_waypoint

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

look_ahead_dist = 1.3  # look-ahead distance [meter]
spacing = 0.6       # distance between lines 
vel_const = 0.2     # [meter/sec]
xy_tolerance = 0.2  # [meter]
yaw_tolerance = 40.0/180.0 * np.pi # [radians]

# gain
kp = 0.7
kd = 0.7

class look_ahead():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_goal = []
        self.x = 0
        self.y = 0
        self.q = np.empty(4)
        self.yaw = 0
        self.pre_steering = 0

        rospy.init_node('look_ahead_following')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function, receive /odom mesage
        rospy.Subscriber('/gnss_imu', Odometry, self.odom_callback, queue_size = 1)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_callback)
        self.pub = rospy.Publisher('/sim_ajk/diff_drive_controller/cmd_vel', Twist, queue_size = 1)
        self.twist = Twist()
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # vehicle's quaternion data in /odom (odometry of ROS message)
        self.q[0] = msg.pose.pose.orientation.x
        self.q[1] = msg.pose.pose.orientation.y
        self.q[2] = msg.pose.pose.orientation.z
        self.q[3] = msg.pose.pose.orientation.w
        yaw = euler_from_quaternion((self.q[0], self.q[1], self.q[2], self.q[3]))[2]
        
        if yaw < 0:    # yaw angle, 0~2pai radian (0~360 degree)
            self.yaw = yaw + 2*np.pi
        else:
            self.yaw = yaw

    # truth position of simulator
    def truth_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "sim_ajk":
                self.x = msg.pose[i].position.x
                self.y = msg.pose[i].position.y
                # vehicle's quaternion data in /odom (odometry of ROS message)
                self.q[0] = msg.pose[i].orientation.x
                self.q[1] = msg.pose[i].orientation.y
                self.q[2] = msg.pose[i].orientation.z
                self.q[3] = msg.pose[i].orientation.w
                yaw = euler_from_quaternion((self.q[0], self.q[1], self.q[2], self.q[3]))[2]
        
                if yaw < 0:    # yaw angle, 0~2pai radian (0~360 degree)
                    self.yaw = yaw + 2*np.pi
                else:
                    self.yaw = yaw

    def shutdown(self):
        print "shutdown"

    def loop(self):
        seq = 1
        while not rospy.is_shutdown():
            # if a specific variable is exists, the proceeds 
            try:
                own_x = self.x
                own_y = self.y
                front_yaw = self.yaw
            except AttributeError:
                continue

            # waypoint with xy coordinate origin adjust
            if seq == 0:
                wp_x_adj = self.waypoint_x[seq] - own_x
                wp_y_adj = self.waypoint_y[seq] - own_y
                own_x_adj = 0
                own_y_adj = 0
            else:
                wp_x_adj = self.waypoint_x[seq] - self.waypoint_x[seq-1]
                wp_y_adj = self.waypoint_y[seq] - self.waypoint_y[seq-1]
                own_x_adj = own_x - self.waypoint_x[seq-1]
                own_y_adj = own_y - self.waypoint_y[seq-1]

            # coordinate transformation of waypoint
            tf_angle = np.arctan2(wp_y_adj, wp_x_adj)
            wp_x_tf = wp_x_adj*np.cos(-tf_angle) - wp_y_adj*np.sin(-tf_angle)
            wp_y_tf = wp_x_adj*np.sin(-tf_angle) + wp_y_adj*np.cos(-tf_angle)

            # coordinate transformation of own position
            front_yaw_tf = front_yaw - tf_angle
            if front_yaw_tf < 0:
                front_yaw_tf = front_yaw_tf + 2*np.pi
            rear_yaw_tf = front_yaw_tf + np.pi
            if rear_yaw_tf > 2*np.pi:
                rear_yaw_tf = rear_yaw_tf -2*np.pi

            # calculate the distance of target line
            u = np.array([wp_x_tf, wp_y_tf])
            v = np.array([own_x, own_y])
            d = np.cross(u, v) / np.linalg.norm(u)

            # calculate the target-angle(bearing) using look-ahead distance
            bearing = np.arctan2(d, look_ahead_dist)            

            # calculate the minimal yaw error, and decide the forward or backward
            front_list = np.empty(3)
            front_list[0] = bearing -front_yaw_tf
            front_list[1] = bearing -front_yaw_tf -2*np.pi
            front_list[2] = bearing -front_yaw_tf +2*np.pi 
            front_steering_ang = front_list[np.argmin(np.abs(front_list))] # min yaw error is selected

            rear_list = np.empty(3)
            rear_list[0] = bearing -rear_yaw_tf
            rear_list[1] = bearing -rear_yaw_tf -2*np.pi
            rear_list[2] = bearing -rear_yaw_tf +2*np.pi 
            rear_steering_ang = rear_list[np.argmin(np.abs(rear_list))] # min yaw error is selected

            if abs(front_yaw_error) > abs(rear_yaw_error):
                steering_ang = rear_steering_ang
                velocity = -vel_const
            elif abs(front_yaw_error) < abs(rear_yaw_error):
                steering_ang = front_steering_ang
                velocity = vel_const

            # calculate the steering_value
            steering_value = kp*steering_ang -kd*pre_steering_ang
            pre_steering_ang = steering_ang

            print wp_x_adj, wp_y_adj, tf_angle/np.pi*180
            print "transform_x:", wp_x_tf, "transform_y:", wp_y_tf
            print "target_line_error:", d

            # If the yaw error is large, pivot turn.
            #if abs(yaw_error) > yaw_tolerance:
            #    self.twist.linear.x = 0
            #    self.twist.angular.z = target_ang
            #else:
            #    self.twist.linear.x = velocity
            #    self.twist.angular.z = target_ang
            #self.pub.publish(self.twist)

            # when reaching the look-ahead distance, read the next waypoint.
            if waypoint_dist < xy_tolerance:
                seq = seq + 1

            if seq >= len(self.waypoint_x):
                self.twist.linear.x = 0
                self.twist.angular.z = 0                
                break
            time.sleep(0.01)

    # load waypoint list
    def load_waypoint(self):
        self.waypoint_x, self.waypoint_y = load_waypoint.load_csv()
        #print self.waypoint_x, self.waypoint_y
    
if __name__ == '__main__':
    l = look_ahead()
    l.load_waypoint()
    l.loop()
