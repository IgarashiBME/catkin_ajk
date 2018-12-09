#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import serial
import sys
import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry

# Protocol of Sanyokiki weeder
StartByte_1 = "S"
StartByte_2 = "T"
ForwardBackward_Neutral = 512 # neutral value
LeftRight_Neutral = 512       # neutral value
EngineSpeed = "0E0E"          # minimam speed is 0A49
EngineOn = "0000"             # engine off value, engine on value is "0080"
AutonomousOff = "0000"
MaxSpeed_Limit = "0050"
CorrectionDataA = "0000"
CorrectionDataB = "0000"
LineFeed = chr(0x0d)
CarriageReturn = chr(0x0a)
CommandLength = 36            # command of sanyokiki weeder is 36 length.

Control_Max = 852
Control_Min = 172
Left_Opt = 464
Right_Opt = 560
SleepConstant = 0.050 # second
GNSS_Freq = 0.24

Kp = 0.5

class controller():
    def __init__(self):
        self.linear_cmd = 0
        self.angular_cmd = 0

        self.cmd_vel_time = 0
        self.safetystop()

        # initialize serial port
        try:
            self.ser=serial.Serial(
                port = '/dev/serial/by-id/usb-TOCOS_TWE-Lite-R_AHXGUC5S-if00-port0',
                baudrate = 115200,
                timeout = 0.05,
                writeTimeout = 0.05,
            )
        except serial.serialutil.SerialException:
            rospy.logerr("port not found")
            sys.exit(0)

        rospy.on_shutdown(self.shutdown)
        print "ready to controller"
        # ROS callback
        rospy.Subscriber('engine_onoff', Int8, self.sendbot, queue_size=1)
        rospy.Subscriber('/sim_ajk/diff_drive_controller/cmd_vel', Twist, self.sendbot_cmd, queue_size=1)
        rospy.Subscriber('imu/data', Imu, self.imu)

    def send_sanyocontroller(self, ForwardBackward, LeftRight, EngineOn):
        self.ControlCommand = StartByte_1 +StartByte_2 +ForwardBackward +LeftRight +EngineSpeed \
                              +EngineOn +AutonomousOff +MaxSpeed_Limit +CorrectionDataA \
                              +CorrectionDataB +LineFeed +CarriageReturn

        t1 = time.time()
        for i in range(CommandLength):
            self.ser.write(self.ControlCommand[0+i:1+i])
            self.ser.flush()
            #print self.ControlCommand[0+i:1+i] #debug option

        t2 = time.time()
        sleep_time = SleepConstant - (t2 - t1)
        if sleep_time > 0:
            time.sleep(sleep_time)
        # debug option
        #t3 = time.time()
        #print (t3-t1)
        #print self.ControlCommand

    def sendbot_cmd(self, new_cmd):
        self.cmd_vel_time = rospy.Time.now().secs
        self.angular_cmd = new_cmd.angular.z
        self.linear_cmd  = new_cmd.linear.x

    def sendbot(self, engine):
        if engine.data == 80:
            self.EngineOn = '00' + str(engine.data)
        else:
            self.EngineOn = '0000'

        # safety stop by detecting /cmd_vel interruption
        t = rospy.Time.now().secs
        if t - self.cmd_vel_time > 1:
            self.safetystop()

            if t - self.cmd_vel_time < 3:
                print "safety stop by detecting /cmd_vel interruption"

        # statement of serial write
        self.send_sanyocontroller(str(self.FB_hex), str(self.LR_hex), self.EngineOn)

    def imu(self, msg):
        # vehicle's angular velocity
        self.imu_angular = msg.angular_velocity.z

    def safetystop(self):
        self.FB_hex = '0' +hex(ForwardBackward_Neutral)[2:5]
        self.LR_hex = '0' +hex(LeftRight_Neutral)[2:5]

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.angular_cmd
                self.linear_cmd
                self.imu_angular
            except AttributeError:
                continue

            # pivot turn
            if self.linear == 0 and self.angular > 0:
                self.ForwardBackward_value = ForwardBackward_Neutral
                self.LeftRight_value = Control_Max
                to_hex()
            elif self.linear == 0 and self.angular < 0:
                self.ForwardBackward_value = ForwardBackward_Neutral
                self.LeftRight_value = Control_Min
                to_hex()
            # PI control
            else:
                power = p(self.imu_angular, self.angular_cmd)               
                On_Time  = GNSS_Freq * power
                Off_Time = GNSS_Freq * (1 - power)
                if self.linear > 0:
                    self.ForwardBackward_value = Control_Max
                    self.LeftRight_value = LeftRight_Neutral
                elif self.linear < 0:
                    self.ForwardBackward_value = Control_Min
                    self.LeftRight_value = LeftRight_Neutral
                to_hex()
                time.sleep(Off_Time)   

                if self.angular_cmd >= 0:
                    self.LeftRight_value = Right_Opt
                elif self.angular_cmd <= 0:
                    self.LeftRight_value = Left_Opt
                to_hex()
                time.sleep(On_Time)               

    def to_hex():
        self.FB_hex = '0' +hex(self.ForwardBackward_value)[2:5]
        self.LR_hex = '0' + hex(self.LeftRight_value)[2:5]
        # upper case is required for sanyo command
        if self.FB_hex.islower() == True:
            self.FB_hex = self.FB_hex.upper()
        if self.LR_hex.islower() == True:
            self.LR_hex = self.LR_hex.upper()

    def p(self, current, target):
        d = abs(target - current)
        power = d / target * Kp
        if power > 1:
            power = 1

        return power

    def shutdown(self):
        rospy.loginfo("sanyo controller was terminated")
    
if __name__ == '__main__':
    rospy.init_node('sanyo_controller')
    c = controller()
    c.loop()
