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

la_dist_const = 1.3  # look-ahead distance [meter]
spacing = 0.6 # distance between lines 
vel_const = 0.2 # [meter/sec]
yaw_tolerance = 40.0/180.0 * np.pi # [radians]
xy_tolerance = 0.2

class pure_pursuit():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_goal = []
        self.x = 0
        self.y = 0
        self.q = np.empty(4)
        self.yaw = 0

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
        seq = 3
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

            # calculation of look-ahead target
            if seq == 0:
                target_x = self.waypoint_x[seq]
                target_y = self.waypoint_y[seq]
            else:
                x1 = self.waypoint_x[seq-1]
                x2 = self.waypoint_x[seq]
                y1 = self.waypoint_y[seq-1]
                y2 = self.waypoint_y[seq]

                # y = ax+b
                if abs(x2-x1) < abs(y2-y1):
                    m = (x2-x1)/(y2-y1)
                    n = (y2*x1 -y1*x2)/(y2-y1)

                    sign = (x2-x1)/abs(x2-x1)
                    for i in range(20):
                        target_y = y1 - abs(m * i) * sign
                        target_x = m *target_y + n
                        #print m, (y1 -y2)
                        print target_x, target_y

                # x = ay+b
                elif abs(x2-x1) > abs(y2-y1):
                    m = (y2-y1)/(x2-x1)
                    n = (x2*y1 -x1*y2)/(x2-x1)

                    sign = (y2-y1)/abs(y2-y1)
                    for i in range(20):
                        target_x = x1 - abs(m * i) * sign
                        target_y = m *target_x + n
                        #print m, (x1 -x2)
                        print target_x, target_y
            exit()

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

            # when reaching the look-ahead distance, read the next waypoint.
            if waypoint_dist < xy_torelance:
                seq = seq + 1
            if seq >= len(self.waypoint_x):
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.pub.publish(self.twist)                
                break
            time.sleep(0.01)

    # load waypoint list
    def load_waypoint(self):
        self.waypoint_x, self.waypoint_y = load_waypoint.load_csv()
        #print self.waypoint_x, self.waypoint_
    
if __name__ == '__main__':
    p = pure_pursuit()
    p.load_waypoint()
    p.loop()
