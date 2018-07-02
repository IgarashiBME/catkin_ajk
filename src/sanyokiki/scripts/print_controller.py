#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import serial
import sys
import time
import datetime
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from sanyokiki.msg import HumanProximity

# Protocol of Sanyokiki weeder
StartByte_1 = "S"
StartByte_2 = "T"
ForwardBackward_Neutral = 512 # neutral value
LeftRight_Neutral = 512       # neutral value
EngineSpeed = "0A49"          # minimam speed
EngineOn = "0000"             # engine off value, engine on value is "0080"
AutonomousOff = "0000"
MaxSpeed_Limit = "0041"
CorrectionDataA = "0000"
CorrectionDataB = "0000"
LineFeed = chr(0x0d)
CarriageReturn = chr(0x0a)
CommandLength = 36            # command of sanyokiki weeder is 36 length.

PowerAdjust = 320


class controller():
    def __init__(self):
        self.human_proximity_stamp = 0
        self.human_proximity_value = 0
        self.cmd_vel_time = 0
        self.safetystop()

        print "ready to controller"
        rospy.Subscriber('engine_onoff', Int8, self.sendbot)    # ROS callback function of engine command
        rospy.Subscriber('cmd_vel', Twist, self.sendbot_cmd)    # ROS callback function of cmd_vel
        rospy.Subscriber('human_proximity', HumanProximity, self.proximity)    # ROS callback function
        rospy.spin()

    def proximity(self, data):
        self.human_proximity_stamp = data.header.stamp.secs
        self.human_proximity_value = data.proximity_value

    def send_sanyocontroller(self, ForwardBackward, LeftRight, EngineOn):
        ControlCommand = StartByte_1 +StartByte_2 +ForwardBackward +LeftRight +EngineSpeed +EngineOn +AutonomousOff +MaxSpeed_Limit +CorrectionDataA +CorrectionDataB +LineFeed +CarriageReturn

        now = []
        t1 = time.time()
        for i in range(CommandLength):
            #t = datetime.datetime.now() #debug option
            #now.append(t.microsecond) #debug option
            #ser.flushInput()
            #ser.write(ControlCommand[0+i:1+i])
            #ser.flush()
            #print ControlCommand[0+i:1+i] #debug option
            time.sleep(0.001) # need 1 msec sleep
        print ControlCommand #debug option
        #print now #debug option

        #t2 = time.time()
        #print (t2 - t1)*1000, "msec"

    def sendbot_cmd(self, new_cmd):
        Power_linear = 0
        Power_angular = 0
        ForwardBackward_value = 0
        LeftRight_value = 0

        self.cmd_vel_time = rospy.Time.now().secs
 
        # turning
        if(new_cmd.linear.x == 0):
            Power_angular = int(PowerAdjust * math.fabs(new_cmd.angular.z)) 
            if(new_cmd.angular.z > 0):
                self.ForwardBackward_value = ForwardBackward_Neutral
                self.LeftRight_value = LeftRight_Neutral +Power_angular
            else:
                self.ForwardBackward_value = ForwardBackward_Neutral
                self.LeftRight_value = LeftRight_Neutral -Power_angular
        # Forward
        elif(new_cmd.linear.x > 0):
            Power_linear = int(PowerAdjust * math.fabs(new_cmd.linear.x))
            Power_angular = int(PowerAdjust * math.fabs(new_cmd.angular.z))
            if(new_cmd.angular.z > 0):
                self.ForwardBackward_value = ForwardBackward_Neutral +Power_linear
                self.LeftRight_value = int(LeftRight_Neutral +Power_angular)
            elif(new_cmd.angular.z < 0):
                self.ForwardBackward_value = ForwardBackward_Neutral +Power_linear
                self.LeftRight_value = int(LeftRight_Neutral -Power_angular)
            else:
                self.ForwardBackward_value = ForwardBackward_Neutral +Power_linear
                self.LeftRight_value = LeftRight_Neutral
        # Backward
        elif(new_cmd.linear.x < 0):
            Power_linear = int(PowerAdjust * math.fabs(new_cmd.linear.x))
            Power_angular = int(PowerAdjust * math.fabs(new_cmd.angular.z))
            if(new_cmd.angular.z > 0):
                self.ForwardBackward_value = ForwardBackward_Neutral -Power_linear
                self.LeftRight_value = int(LeftRight_Neutral +Power_angular)
            elif(new_cmd.angular.z < 0):
                self.ForwardBackward_value = ForwardBackward_Neutral -Power_linear
                self.LeftRight_value = int(LeftRight_Neutral -Power_angular)
            else:
                self.ForwardBackward_value = ForwardBackward_Neutral -Power_linear
                self.LeftRight_value = LeftRight_Neutral 

        # safety stop by detecting human
        t = rospy.Time.now()
        if self.human_proximity_value == 1 and t.secs - self.human_proximity_stamp < 3:
            self.safetystop()
            if t.secs - self.human_proximity_stamp > 0.5:
                print "human_detected" + str(t.secs)
        else:
            self.ForwardBackward_value = '0' +hex(self.ForwardBackward_value)[2:5]
            self.LeftRight_value = '0' + hex(self.LeftRight_value)[2:5]
            # upper case is required for sanyo command
            if self.ForwardBackward_value.islower() == True:
                self.ForwardBackward_value = self.ForwardBackward_value.upper()
            if self.LeftRight_value.islower() == True:
                self.LeftRight_value = self.LeftRight_value.upper()


    def sendbot(self, engine):
        """try:
            self.ForwardBackward_value = self.ForwardBackward_value
    
        except NameError:
            self.ForwardBackward_value = '0' +hex(ForwardBackward_Neutral)[2:5]
            self.LeftRight_value = '0' +hex(LeftRight_Neutral)[2:5]"""

        if engine.data == 80:
            EngineOn = '00' + str(engine.data)
        else:
            EngineOn = '0000'

        # safety stop by detecting /cmd_vel interruption
        t = rospy.Time.now().secs
        if t - self.cmd_vel_time > 1:
            self.safetystop()

            if t - self.cmd_vel_time < 3:
                print "safety stop by detecting /cmd_vel interruption"

        # statement of serial write
        self.send_sanyocontroller(self.ForwardBackward_value, self.LeftRight_value, EngineOn)

    def safetystop(self):
        self.ForwardBackward_value = '0' +hex(ForwardBackward_Neutral)[2:5]
        self.LeftRight_value = '0' +hex(LeftRight_Neutral)[2:5]

def shutdown():
    rospy.loginfo("sanyo controller was terminated")
    #ser.close()
    
# main
if __name__ == '__main__':
    rospy.init_node('sanyo_controller')
    rospy.on_shutdown(shutdown)
    c = controller()
