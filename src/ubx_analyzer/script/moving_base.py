#!/usr/bin/python
# coding:utf-8

import serial
import rospy
import binascii
import struct
import numpy as np

# ROS message form
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ubx_analyzer.msg import NAV-RELPOSNED

Header_A = "b5"  #Ublox GNSS receiver, UBX protocol header
Header_B = "62"  #Ublox GNSS receiver, UBX protocol header
NAV_ID = "01"    #UBX-NAV-RELPOSNED header(0x01 0x3c)
RELPOSNED_ID = "3c"
NAV_RELPOSNED_Length = 40    #length of UBX-NAV-RELPOSNED hex data

class ublox():
    def __init__(self):
        # Set up serial:
        self.ser = serial.Serial(
            port='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',\
            baudrate=38400,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=1)

        rospy.on_shutdown(self.shutdown)

        # ROS publisher initialize
        self.pub_moving_base = rospy.Publisher('moving_base', Imu, queue_size = 1)
        self.moving_base = Imu()
        self.pub_relposned = rospy.Publisher('relposned', NAV-RELPOSNED, queue_size = 1)
        self.relposned = NAV-RELPOSNED()

    def loop(self):
        # searching for UBX-NAV-Class headers
        while not rospy.is_shutdown():
            Data = binascii.b2a_hex(self.ser.read())

            if Data == Header_A:
                Data = binascii.b2a_hex(self.ser.read())
 
                if Data == Header_B:
                    Data = binascii.b2a_hex(self.ser.read())

                    if Data == NAV_ID:
                        Data = binascii.b2a_hex(self.ser.read())

                        if Data == RELPOSNED_ID:
                            NAV_RELPOSENED_Data = bytearray(self.ser.read(NAV_RELPOSNED_Length))
                            #print binascii.b2a_hex(NAV_RELPOSENED_Data)
                            self.RELPOSNED_Function(NAV_RELPOSENED_Data)

    def HPPOSLLH_Function(self, Data):
        #print binascii.b2a_hex(Data)
        if Data.__len__() == NAV_RELPOSNED_Length:
            # GPS time of week
            gpst = float(struct.unpack('I', struct.pack('BBBB', Data[6], Data[7],
                                       Data[8], Data[9]))[0])

            # relative position vector (unit: centi-meter)
            # relPosN: North component
            # relPosE: East component
            # relPosD: Down component
            relPosN = float(struct.unpack('i', struct.pack('BBBB', Data[10],
                                            Data[11], Data[12], Data[13]))[0])
            relPosE = float(struct.unpack('i', struct.pack('BBBB', Data[14], 
                                            Data[15], Data[16], Data[17]))[0])
            relPosD = float(struct.unpack('i', struct.pack('BBBB', Data[18], Data[19], 
                                         Data[20], Data[21]))[0])

            # High Precision of relative position vector
            relPosHPN = float(struct.unpack('b', struct.pack('B', Data[22]))[0])/100.0
            relPosHPE = float(struct.unpack('b', struct.pack('B', Data[23]))[0])/100.0
            relPosHPD = float(struct.unpack('b', struct.pack('B', Data[24]))[0])/100.0

            # Accuracy of relative position
            accN = float(struct.unpack('I', struct.pack('BBBB', Data[26], Data[27],
                                                        Data[28], Data[29]))[0])
            accE = float(struct.unpack('I', struct.pack('BBBB', Data[30], Data[31],
                                                        Data[32], Data[33]))[0])
            accD = float(struct.unpack('I', struct.pack('BBBB', Data[34], Data[35],
                                                        Data[36], Data[37]))[0])

            # calculate heading from 
            heading = np.arctan2((relPosN+relPosHPN), (relPosE+relPosHPE))

            # Publish RELPOSNED
            self.relposned.iTOW = gpst
            self.relposned.fix_status = self.fix_status
            self.relposned.relPosN = relPosN
            self.relposned.relPosE = relPosE
            self.relposned.relPosD = relPosD
            self.relposned.relPosHPN = relPosN
            self.relposned.relPosHPE = relPosE
            self.relposned.relPosHPD = relPosD
            self.relposned.accN = accN
            self.relposned.accE = accE
            self.relposned.accD = accD

            #print gpst
            #print 
            #print    

    def shutdown(self):
        rospy.loginfo("ublox analyzer node was terminated") 

if __name__ == '__main__':
    rospy.init_node("ublox_moving_base_node")
    u = ublox()
    u.loop()
        
