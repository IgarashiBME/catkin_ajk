#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# using GNSS UTM cordinate date

import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates

def callback(msg):
    for i, name in enumerate(msg.name):
        if name == "sim_ajk":
            print msg.pose[i]

def shutdown():
    rospy.loginfo("sub_robot_node was terminated")

def listener():
    rospy.init_node('sub_robot_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
