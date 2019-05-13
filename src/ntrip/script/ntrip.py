#!/usr/bin/python
# -*- coding: utf-8 -*- 

import rospy
import socket
import base64
import sys
import binascii
import time

from std_msgs.msg import String

class ntrip():
    def __init__(self):
        # ROS publisher
        self.pub_rtcm = rospy.Publisher('ntrip_rtcm', String, queue_size = 1)

        # ROS subscriber
        rospy.Subscriber('/gngga', String, self.gngga_subs, queue_size=1)

        while rospy.is_shutdown():
            rospy.loginfo("waiting")

        address = rospy.get_param('~ntrip_address')
        port = rospy.get_param('~ntrip_port')
        username = rospy.get_param('~ntrip_username')
        password = rospy.get_param('~ntrip_password')
        mountpoint = rospy.get_param('~ntrip_mountpoint')

        pwd = base64.b64encode("{}:{}".format(username, password).encode('ascii'))

        header =\
        "GET /{} HTTP/1.1\r\n".format(mountpoint) +\
        "Host \r\n".format("130.69.124.13") +\
        "Ntrip-Version: Ntrip/2.0\r\n" +\
        "User-Agent: NTRIP sho/0.0\r\n" +\
        "Accept: */*" +\
        "Connection: close\r\n" +\
        "Authorization: Basic {}\r\n\r\n".format(pwd)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((address,int(port)))
        self.s.send(header)

        resp = self.s.recv(1024)

        if resp.startswith("STREAMTABLE"):
            raise NTRIPError("Invalid or No Mountpoint")
        elif not resp.startswith("HTTP/1.1 200 OK"):
            raise NTRIPError("Invalid Server Response")

    def gngga_subs(self, msg):
        self.gngga = msg.data

    def loop(self):
        while not rospy.is_shutdown():
            #dummy_nmea = "$GNGGA,023849.40,3542.99814,N,13945.70730,E,1,06,3.18,29.1,M,39.4,M,,*7E\r\n"
            try:
                self.gngga
            except:
                continue

            self.s.send(self.gngga.encode('ascii'))

            time.sleep(0.01)

            data = self.s.recv(1024)
            #print(binascii.b2a_hex(data))

            self.pub_rtcm.publish(binascii.b2a_hex(data))
            sys.stdout.flush()

if __name__ == '__main__':
    rospy.init_node("ntrip_node")
    n = ntrip()
    n.loop()
