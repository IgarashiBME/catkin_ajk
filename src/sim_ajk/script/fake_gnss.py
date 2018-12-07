#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# using GNSS UTM cordinate date

import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

utm = Odometry()
pub_utm = rospy.Publisher('utm', Odometry, queue_size = 1)

def callback(msg):
    for i, name in enumerate(msg.name):
        if name == "sim_ajk":
            #print msg.pose[i].position
            utm.pose.pose.position.x = msg.pose[i].position.x
            utm.pose.pose.position.y = msg.pose[i].position.y
            utm.pose.pose.position.z = msg.pose[i].position.z
            pub_utm.publish(utm)

def shutdown():
    rospy.loginfo("sub_robot_node was terminated")

def listener():
    rospy.init_node('fake_gnss_sim_ajk_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
