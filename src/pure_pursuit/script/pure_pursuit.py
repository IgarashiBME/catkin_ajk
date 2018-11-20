#! /usr/bin/env python

import rospy
import time
import numpy as np
import csv
import os

import load_waypoint

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class pure_pursuit():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.q = np.empty(4)
        self.x = 0
        self.y = 0

        rospy.init_node('pure_pursuit_control')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function, receive /odom mesage
        rospy.Subscriber('/sim_create/diff_drive_controller/odom', Odometry, self.odom_callback) 
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # vehicle's quaternion data in /odom (odometry of ROS message)
        self.q[0] = msg.pose.pose.orientation.x
        self.q[1] = msg.pose.pose.orientation.y
        self.q[2] = msg.pose.pose.orientation.z
        self.q[3] = msg.pose.pose.orientation.w

    def shutdown(self):
        print "shutdown"

    def loop(self):
        seq = 0
        while not rospy.is_shutdown():

            # Confirm the existence of self.x brought by the odom_callback
            try:
                self.x
            except AttributeError:
                continue

            a = np.array([self.waypoint_x[seq], self.waypoint_y[seq]])
            b = np.array([self.x, self.y])
            waypoint_dist = np.linalg.norm(b-a)

            print waypoint_dist
            seq = seq + 1
            if seq >= len(self.waypoint_x):
                break

    # load waypoint list
    def load_waypoint(self):
        x, y = load_waypoint.load_csv()
        self.waypoint_x, self.waypoint_y = load_waypoint.interpolation(x, y)
        #print self.waypoint_x, self.waypoint_y        
    
if __name__ == '__main__':
    p = pure_pursuit()
    p.load_waypoint()
    p.loop()
