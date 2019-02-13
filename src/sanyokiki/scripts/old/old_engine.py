#!/usr/bin/env python
# coding:utf-8

#title              :engine.py
#description        :This script is a ROS python script.
#                    By pressing the keyboard, the engine of the weeder is turned on or off.
#author             :Sho Igarashi
#date               :20180608

import rospy
import select
import sys
import time

from std_msgs.msg import Int8

# Main program loop:
def engine():
    engine_onoff = 0
    rospy.init_node("engine_node")
    print "ready to engine message"
    messages()
    pub_engine = rospy.Publisher('engine_onoff', Int8, queue_size = 1)

    # ROS publisher initialize
    while not rospy.is_shutdown():
        kb = input()
    
        if str(kb) == 'q':
            print("exit")
            exit()

        elif str(kb) == "o":
            print("The engine was turned on")
            messages()
            engine_onoff = 80

        elif str(kb) == "p":
            print("The engine was turned off")
            messages()
            engine_onoff = 0

        pub_engine.publish(engine_onoff)
        time.sleep(0.01)

def messages():
    print "o: engine on,  p: engine off,  q: exit"

def input():
    i,o,e = select.select([sys.stdin],[],[], 0.04)

    if (i):
        key = sys.stdin.read(1)
        print key
        return key
    else:
        return None   

if __name__ == '__main__':
    try:
        engine()
    except rospy.ROSInterruptException: pass
        
