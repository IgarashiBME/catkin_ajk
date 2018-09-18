#!/usr/bin/python
# coding:utf-8

import serial
import rospy
import binascii
import struct
from pyproj import Proj

from sensor_msgs.msg import NavSatFix    # ROS message form
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ubx_analyzer.msg import NavPVT 


Header_A = "b5"  #Ublox GNSS receiver, UBX protocol header
Header_B = "62"  #Ublox GNSS receiver, UBX protocol header
NAV_Id = "01"    #UBX-NAV-PVT class header(0x01 0x07)
PVT_Id = "07"
NAV_PVT_Length = 96    #length of UBX-NAV-PVT hex data

# Set up serial:
ser = serial.Serial(
    port='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',\
    baudrate=38400,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=1)

# Main program loop:
def ubx():
    rospy.init_node("ublox_analyzer_node")

    # ROS publisher initialize
    pub_ubx = rospy.Publisher("gnss", NavSatFix, queue_size = 1)
    pub_gpst = rospy.Publisher('gpstime', String, queue_size = 1)
    pub_utm = rospy.Publisher('utm', Odometry, queue_size = 1)
    pub_navpvt = rospy.Publisher('navpvt', NavPVT, queue_size = 1)
    utm = Odometry()
    ubx_data = NavSatFix()
    navpvt_data = NavPVT()

    # searching for UBX-NAV-PVT headers
    while not rospy.is_shutdown():
        NAV_PVT_Data = []
        Data = binascii.b2a_hex(ser.read())

        if Data == Header_A:
            Data = binascii.b2a_hex(ser.read())
 
            if Data == Header_B:
                Data = binascii.b2a_hex(ser.read())

                if Data == NAV_Id:
                    Data = binascii.b2a_hex(ser.read())

                    if Data == PVT_Id:
                        NAV_PVT_Data = bytearray(ser.read(NAV_PVT_Length))

        if NAV_PVT_Data.__len__() == 96:
            # decode Time data
            gpst = float(struct.unpack('I', struct.pack('BBBB', NAV_PVT_Data[2], NAV_PVT_Data[3],
                                       NAV_PVT_Data[4], NAV_PVT_Data[5]))[0])
            year = int(struct.unpack('h', struct.pack('BB', NAV_PVT_Data[6], NAV_PVT_Data[7]))[0])
            month = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[8]))[0])
            day = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[9]))[0])
            hour = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[10]))[0])
            minute = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[11]))[0])
            second = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[12]))[0])
            
            # decode RTK fix flag
            fix_flag = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[23]))[0])
            fix_flag = bin(fix_flag)[2:].zfill(8)
            if fix_flag[0:2] == str(10):
                fix_status = 2    # when rtk was fixed, publish 2
                fix_str = "RTK fixed solution"
            elif fix_flag[0:2] == str(01):
                fix_status= 1     # when rtk was float, publish 1
                fix_str = "RTK float solution"
            else:
                fix_status= 0
                fix_str = "No carrier phase renge solution"

            # decode Number of satellites used in Nav Solution
            satellites = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[25]))[0])

            # decode Coordinate data
            longitude = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[26],
                                            NAV_PVT_Data[27], NAV_PVT_Data[28], NAV_PVT_Data[29]))[0])/10000000.0
            latitude = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[30], 
                                            NAV_PVT_Data[31], NAV_PVT_Data[32], NAV_PVT_Data[33]))[0])/10000000.0
            height = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[34], NAV_PVT_Data[35], 
                                         NAV_PVT_Data[36], NAV_PVT_Data[37]))[0])/1000.0

            # publish GNSS positioning data
            ubx_data.header.stamp = rospy.Time.now()
            ubx_data.latitude = latitude
            ubx_data.longitude = longitude
            ubx_data.altitude = height
            ubx_data.status.status = fix_status
            pub_ubx.publish(ubx_data)

            # publish rostime and gpstime
            pub_gpst.publish(str(ubx_data.header.stamp.secs) +"," +str(gpst))

            # publish UTM coordinate
            utmzone = int((longitude + 180)/6) +1   # If you are on the specific location, can't be calculated. 
            convertor = Proj(proj='utm', zone=utmzone, ellps='WGS84')
            x, y = convertor(longitude, latitude)
            utm.header.stamp = rospy.Time.now()
            utm.pose.pose.position.x = x
            utm.pose.pose.position.y = y
            utm.pose.pose.position.z = height
            pub_utm.publish(utm)

            # publish GNSS status
            navpvt_data.iTOW = gpst
            navpvt_data.year = year
            navpvt_data.month = month
            navpvt_data.day = day
            navpvt_data.hour = hour
            navpvt_data.min = minute
            navpvt_data.sec = second
            navpvt_data.numSV = satellites
            navpvt_data.lon = longitude
            navpvt_data.lat = latitude
            navpvt_data.height = height
            pub_navpvt.publish(navpvt_data)            

            # print section
            #print gpst
            time_data = str(year) + "," + str(month) + "," + str(day) + "," + str(hour) \
                        + "," + str(minute) + "," + str(second)
            coordinate_data = str(latitude) + "," + str(longitude) + "," + str(height)
            print time_data
            print fix_str, fix_status, fix_flag[0:2]
            print coordinate_data
            print "east:" + str(x), "north:" + str(y)
            print

if __name__ == '__main__':
    try:
        ubx()
    except rospy.ROSInterruptException: pass
        
