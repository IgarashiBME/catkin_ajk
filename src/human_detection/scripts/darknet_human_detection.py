#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from image_process.msg import BoundingBoxes,BoundingBox,HumanProximity
import numpy as np

pub_human_proximity = rospy.Publisher('human_proximity', HumanProximity, queue_size = 1)
proximity = HumanProximity()

def callback_bounding_boxes(data):
    for i in data.boundingBoxes:
        if i.Class == "person":
            proximity.header.stamp = rospy.Time.now()
            proximity.proximity_value = 1                
            pub_human_proximity.publish(proximity)

def listener():
    rospy.init_node('darknet_human_detection', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_bounding_boxes)
    rospy.spin()

if __name__ == '__main__':
    listener()
