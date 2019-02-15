#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
import sys
import select
import termios
import time

from std_msgs.msg import Int8
from sanyokiki.msg import AJK_value

FORWARD_CONST = 732
BACKWARD_CONST = 292
RIGHT_CONST = 332
LEFT_CONST = 692
LOW_RIGHT_CONST = 463
LOW_LEFT_CONST = 561
NEUTRAL_CONST = 512

def teleop():
    engine_onoff = 0
    rospy.init_node("engine_node")
    print "ready to engine message"
    messages()
    pub_engine = rospy.Publisher('/engine_onoff', Int8, queue_size = 1)
    pub_ajk_manual = rospy.Publisher('/ajk_manual', AJK_value, queue_size = 1)
    ajk_manual = AJK_value()

    # ROS publisher initialize
    while not rospy.is_shutdown():
        kb = input()
    
        if str(kb) == '0':
            print("exit")
            exit()

        elif str(kb) == "h":
            messages()

        elif str(kb) == "o":
            print("The engine was turned on")
            engine_onoff = 80

        elif str(kb) == "p":
            print("The engine was turned off")
            engine_onoff = 0

        elif str(kb) == "w":    # forward
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = FORWARD_CONST
            ajk_manual.steering = NEUTRAL_CONST
            pub_ajk_manual.publish(ajk_manual)

        elif str(kb) == "a":    # left
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = NEUTRAL_CONST
            ajk_manual.steering = LEFT_CONST
            pub_ajk_manual.publish(ajk_manual)

        elif str(kb) == "s":    # backward
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = BACKWARD_CONST
            ajk_manual.steering = NEUTRAL_CONST
            pub_ajk_manual.publish(ajk_manual)

        elif str(kb) == "d":    # right
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = NEUTRAL_CONST
            ajk_manual.steering = RIGHT_CONST
            pub_ajk_manual.publish(ajk_manual)

        elif str(kb) == "e":    # right forward
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = FORWARD_CONST
            ajk_manual.steering = LOW_RIGHT_CONST
            pub_ajk_manual.publish(ajk_manual)

        elif str(kb) == "q":    # left forward
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = FORWARD_CONST
            ajk_manual.steering = LOW_LEFT_CONST
            pub_ajk_manual.publish(ajk_manual)

        elif str(kb) == "z":    # left backward
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = BACKWARD_CONST
            ajk_manual.steering = LOW_LEFT_CONST
            pub_ajk_manual.publish(ajk_manual)

        elif str(kb) == "c":    # right backward
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = BACKWARD_CONST
            ajk_manual.steering = LOW_RIGHT_CONST
            pub_ajk_manual.publish(ajk_manual)

        """elif kb == None:
            ajk_manual.stamp = rospy.Time.now()
            ajk_manual.translation = 0
            ajk_manual.steering = 0
            pub_ajk_manual.publish(ajk_manual)"""

        pub_engine.publish(engine_onoff)
        time.sleep(0.01)

def messages():
    print "0: exit, h: help"
    print "o: engine on,  p: engine off"

def input():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~termios.ICANON
    new[3] &= ~termios.ECHO

    try:
        termios.tcsetattr(fd, termios.TCSANOW, new)
        i,o,e = select.select([sys.stdin],[],[], 0.1)
        if (i):
            key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)

    if (i):
        return key
    else:
        return None   

if __name__=="__main__":
    teleop()
