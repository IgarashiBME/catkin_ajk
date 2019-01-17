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

la_dist_const = 0.6  # look-ahead distance [meter]
spacing = 0.6 # distance between lines 
vel_const = 0.2 # [meter/sec]
yaw_tolerance = 40.0/180.0 * np.pi # [radians]

class pure_pursuit():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_goal = []
        self.x = 0
        self.y = 0
        self.q = np.empty(4)
        self.yaw = 0

        self.la_dist = la_dist_const

        rospy.init_node('pure_pursuit_control')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function, receive /odom mesage
        rospy.Subscriber('/gnss_imu', Odometry, self.odom_callback, queue_size = 1)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_callback)
        self.pub = rospy.Publisher('/sim_ajk/diff_drive_controller/cmd_vel', Twist, queue_size = 1)
        self.twist = Twist()
        self.pubstr = rospy.Publisher('/straight_str', String, queue_size = 1)
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # vehicle's quaternion data in /odom (odometry of ROS message)
        self.q[0] = msg.pose.pose.orientation.x
        self.q[1] = msg.pose.pose.orientation.y
        self.q[2] = msg.pose.pose.orientation.z
        self.q[3] = msg.pose.pose.orientation.w
        self.yaw  = euler_from_quaternion((self.q[0], self.q[1], self.q[2], self.q[3]))[2]

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
                self.yaw  = euler_from_quaternion((self.q[0], self.q[1], self.q[2], self.q[3]))[2]

    def shutdown(self):
        print "shutdown"

    def loop(self):
        seq = 0
        while not rospy.is_shutdown():

            # Confirm the existence of self.x brought by the odom_callback
            try:
                x = self.x
                y = self.y
                front_yaw = self.yaw
            except AttributeError:
                continue

            a = np.array([self.waypoint_x[seq], self.waypoint_y[seq]])
            b = np.array([x, y])
            waypoint_dist = np.linalg.norm(b-a)

            if front_yaw < 0:    # yaw angle, 0~2pai radian (0~360 degree)
                front_yaw = front_yaw + 2*np.pi
            rear_yaw = front_yaw + np.pi
            if rear_yaw > 2*np.pi:
                rear_yaw = rear_yaw -2*np.pi

            # target_yaw
            target_yaw = np.arctan2(self.waypoint_y[seq]-y, self.waypoint_x[seq]-x)
            if target_yaw < 0:   # yaw angle, 0~2pai radian (0~360 degree)
                target_yaw = target_yaw + 2*np.pi

            forward_list = [0,0,0]
            forward_list[0] = target_yaw -front_yaw
            forward_list[1] = target_yaw -(front_yaw+2*np.pi)
            forward_list[2] = target_yaw +2*np.pi -front_yaw
            yaw_error = forward_list[np.argmin(np.abs(forward_list))] # min yaw error is selected

            backward_list = [0,0,0]
            backward_list[0] = target_yaw -rear_yaw
            backward_list[1] = target_yaw -(rear_yaw +2*np.pi)
            backward_list[2] = target_yaw +2*np.pi -rear_yaw
            back_yaw_error = backward_list[np.argmin(np.abs(backward_list))] # min yaw error is selected

            print 
            print x, y
            print self.waypoint_x[seq], self.waypoint_y[seq]
            print "target_yaw:", target_yaw/np.pi*180
            print " front_yaw:", front_yaw/np.pi*180
            print "  rear_yaw:", rear_yaw/np.pi*180
            print "f",forward_list
            print "b",backward_list

            # minimal yaw error is selected
            if abs(yaw_error) > abs(back_yaw_error):
                yaw_error = back_yaw_error
                velocity = -vel_const
            elif abs(yaw_error) < abs(back_yaw_error):
                velocity = vel_const
            target_ang = (2*vel_const*np.sin(yaw_error))/self.la_dist

            print waypoint_dist,target_ang
            #print target_ang

            # If the yaw error is large, pivot turn.
            if abs(yaw_error) > yaw_tolerance:
                self.twist.linear.x = 0
                self.twist.angular.z = target_ang
            else:
                self.twist.linear.x = velocity
                self.twist.angular.z = target_ang
            self.pub.publish(self.twist)

            # If the goal is close, shorten the look-ahead distance
            if self.waypoint_goal[seq] == 1.0:
                self.la_dist = la_dist_const/3
                self.pubstr.publish("not")

            # when reaching the look-ahead distance, read the next waypoint.
            if waypoint_dist < self.la_dist:
                seq = seq + 1
                self.la_dist = la_dist_const
                self.pubstr.publish("straight")
            if seq >= len(self.waypoint_x):
                self.twist.linear.x = 0
                self.twist.angular.z = 0                
                break
            time.sleep(0.01)

    # load waypoint list
    def load_waypoint(self):
        x, y = load_waypoint.load_csv()
        self.waypoint_x, self.waypoint_y, self.waypoint_goal = load_waypoint.interpolation(x, y, spacing)
        print x, y
        #print self.waypoint_goal
    
if __name__ == '__main__':
    p = pure_pursuit()
    p.load_waypoint()
    p.loop()
