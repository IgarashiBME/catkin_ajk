#! /usr/bin/env python

# Calculate route from setted two point
# using GNSS UTM cordinate date

import rospy
import time
import numpy as np
import csv
import os

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

x = 0
y = 0

def writer():
    global x, y

    print "ready to save utm"
    print "press s and enter, save utm as waypoint"
    print "press 0 and enter, exit"

    while not rospy.is_shutdown():
        kb = input()
    
        if kb == 0:
            print("exit")
            exit()
        elif kb == "s":    # forward point
            f = open('route.csv', 'ab')
            csvWriter = csv.writer(f)
            utmdata = []
            utmdata.append(x)
            utmdata.append(y)
            utmdata.append(0)
            csvWriter.writerow(utmdata)
            utmdata = []
            print "x=",x, "y=",y, "backward_frag= 0"
            f.close()
           

def gnss_yaw(utm):
    global x, y
    x = utm.pose.pose.position.x    # read ROS /utm topic data
    y = utm.pose.pose.position.y

def shutdown():
    rospy.loginfo("gnss_yaw_node was terminated")

def listener():
    rospy.init_node('save_utm_waypoint_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('utm', Odometry, gnss_yaw) # ROS callback function
    writer()
    rospy.spin()

if __name__ == '__main__':
    csvdir = os.path.abspath(os.path.dirname(__file__))
    os.chdir(csvdir)
    with open('route.csv', 'w'):
        pass
    listener()
