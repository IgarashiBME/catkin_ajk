#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import serial
import sys
import time
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from sanyokiki.msg import AJK_value
#from sanyokiki.msg import HumanProximity

# Protocol of Sanyokiki weeder
START_BYTE = "ST"
TRANSLATION_NEUTRAL = 512     # neutral value
STEERING_NEUTRAL = 512        # neutral value
ENGINE_SPEED = "0E0E"          # minimam speed is 0A49
ENGINE_ON = "0080"             # engine off value, engine on value is "0080"
ENGINE_OFF = "0000" 
AUTONOMOUS_OFF = "0000"
MAXSPEED_LIMIT = "0050"
CORRECTION_A = "0000"
CORRECTION_B = "0000"
LINE_FEED = chr(0x0d)
CARRIAGE_RETURN = chr(0x0a)
COMMAND_LENGTH = 36            # command of sanyokiki weeder is 36 length

# sleep constant
WRITE_SLEEP = 0.060 # 0.060 seconds

# interval
PRINT_INTERVAL = 1 # 1 seconds
CONTROL_SIGNAL_INTERVAL = 0.3 # The time since the manual and auto signal stopped, 0.3 seconds

class controller():
    def __init__(self):
        #self.human_proximity_stamp = 0
        #self.human_proximity_value = 0
        self.translation_value = '0' +hex(TRANSLATION_NEUTRAL)[2:5]
        self.steering_value = '0' +hex(STEERING_NEUTRAL)[2:5]
        self.manual_stamp = 0
        self.auto_translation_value = '0' +hex(TRANSLATION_NEUTRAL)[2:5]
        self.auto_steering_value = '0' +hex(STEERING_NEUTRAL)[2:5]
        self.auto_stamp = 0

        # initialize serial port
        """try:
            self.ser=serial.Serial(
                port = '/dev/serial/by-id/usb-TOCOS_TWE-Lite-R_AHXGUC5S-if00-port0',
                baudrate = 115200,
                timeout = 0.05,
                writeTimeout = 0.05,
            )
        except serial.serialutil.SerialException:
            rospy.logerr("port not found")
            sys.exit(0)"""

        rospy.on_shutdown(self.shutdown)
        print "ready to controller"
        rospy.Subscriber('/engine_onoff', Int8, self.engine_subs, queue_size=1)  # ROS callback function of engine command
        rospy.Subscriber('/ajk_manual', AJK_value, self.ajk_manual_subs, queue_size=1)
        rospy.Subscriber('/ajk_auto', AJK_value, self.ajk_auto_subs, queue_size=1) 
        #rospy.Subscriber('human_proximity', HumanProximity, self.proximity)    # ROS callback function

    # human procimity checker
    """def proximity(self, data):
        self.human_proximity_stamp = data.header.stamp.secs
        self.human_proximity_value = data.proximity_value"""

    def serial_write(self, translation, steering, engine_status):
        try:
            if self.auto_translation != 0 and self.auto_steering != 0:
                self.ControlCommand = START_BYTE +auto_translation +auto_steering +ENGINE_SPEED +engine_status \
                                      +AUTONOMOUS_OFF +MAXSPEED_LIMIT +CORRECTION_A +CORRECTION_B \
                                      +LINE_FEED +CARRIAGE_RETURN                
        except AttributeError:
            pass
        self.ControlCommand = START_BYTE +translation +steering +ENGINE_SPEED +engine_status \
                              +AUTONOMOUS_OFF +MAXSPEED_LIMIT +CORRECTION_A +CORRECTION_B \
                              +LINE_FEED +CARRIAGE_RETURN

        t1 = time.time()

        # send one character at a time
        for i in range(COMMAND_LENGTH):
            try:
                self.ser.write(self.ControlCommand[0+i:1+i])
                self.ser.flush()
            except AttributeError as e:
                #print e
                pass
            #print self.ControlCommand[0+i:1+i]
            time.sleep(0.0008) # need 0.8 msec sleep

        t2 = time.time()
        sleep_time = WRITE_SLEEP - (t2 - t1)
        if sleep_time > 0:
            #after send all characters, then sleep for the specified time
            time.sleep(sleep_time)

        # debug print
        #t3 = time.time()
        #print (t3-t1)
        #print self.ControlCommand

    def safety_stop(self):
        self.translation_value = '0' +hex(TRANSLATION_NEUTRAL)[2:5]
        self.steering_value = '0' +hex(STEERING_NEUTRAL)[2:5]
        self.auto_translation_value = '0' +hex(TRANSLATION_NEUTRAL)[2:5]
        self.auto_steering_value = '0' +hex(STEERING_NEUTRAL)[2:5]
        print "stop"

    # ROS callback
    def engine_subs(self, msg):
        if msg.data == 80:
            engine_status = ENGINE_ON
        else:
            engine_status = ENGINE_OFF

        now = rospy.Time.now().secs + rospy.Time.now().nsecs/1000000000.0
        if now - self.manual_stamp < CONTROL_SIGNAL_INTERVAL:
            self.serial_write(str(self.translation_value), str(self.steering_value), engine_status)
        elif now - self.auto_stamp < CONTROL_SIGNAL_INTERVAL:
            self.serial_write(str(self.auto_translation_value), str(self.auto_steering_value), engine_status)
        else:
            self.safety_stop()
            self.serial_write(str(self.translation_value), str(self.steering_value), engine_status)            
            
    # ROS callback
    def ajk_manual_subs(self, msg):
        self.translation_value = '{:04x}'.format(msg.translation)
        self.steering_value = '{:04x}'.format(msg.steering)
        self.manual_stamp = msg.stamp.secs +msg.stamp.nsecs/1000000000.0
        
        #print self.manual_stamp
        #print rospy.Time.now().secs + rospy.Time.now().nsecs/1000000000.0

        # upper case is required for sanyo command
        if self.translation_value.islower() == True:
            self.translation_value = self.translation_value.upper()
        if self.steering_value.islower() == True:
            self.steering_value = self.steering_value.upper()

    # ROS callback
    def ajk_auto_subs(self, msg):
        self.auto_translation = '{:04x}'.format(msg.translation)
        self.auto_steering = '{:04x}'.format(msg.steering)
        self.auto_stamp = msg.stamp.secs +msg.stamp.nsecs/1000000000

        # upper case is required for sanyo command
        if self.auto_translation.islower() == True:
            self.auto_translation = self.auto_translation.upper()
        if self.auto_steering.islower() == True:
            self.auto_steering = self.auto_steering.upper()         

    # loginfo loop of control command
    def loop(self):
        t1 = 0
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            try:
                self.ControlCommand
            except AttributeError as e:
                rospy.loginfo(e)
                continue

            # print AJK command with interval 
            t2 = time.time()
            if t2 -t1 > PRINT_INTERVAL:
                rospy.loginfo(self.ControlCommand)
                t1 = time.time()

    def shutdown(self):
        rospy.loginfo("sanyo controller was terminated")
    
if __name__ == '__main__':
    rospy.init_node('sanyo_controller')
    c = controller()
    c.loop()
