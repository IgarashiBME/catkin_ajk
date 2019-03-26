#! /usr/bin/python
# coding:utf-8

import rospy
import rosnode
import subprocess

from mavlink_ajk.msg import MAV_Modes

# MAVLink number
MAV_CMD_NAV_WAYPOINT = 16
ARDUPILOT_AUTO_BASE = 217
ARDUPILOT_AUTO_CUSTOM = 10

class auto_rosbag():
    def __init__(self):
        # mav_modes
        self.mission_start = False
        self.base_mode = 0
        self.custom_mode = 0

        rospy.init_node('auto_rosbag_following')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function, receive /odom mesage
        rospy.Subscriber('/mav/modes', MAV_Modes, self.mav_modes)

    def mav_modes(self, msg):
        self.mission_start = msg.mission_start
        self.base_mode = msg.base_mode
        self.custom_mode = msg.custom_mode

    def shutdown(self):
        print "shutdown"

    def loop(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            isNodeAlive = rosnode.rosnode_ping("/mybag", max_count=1, verbose=False)
            rospy.loginfo(isNodeAlive) 
            if self.base_mode == ARDUPILOT_AUTO_BASE and self.custom_mode == ARDUPILOT_AUTO_CUSTOM:
                if isNodeAlive == False:
                    p=subprocess.Popen(["rosbag", "record", "-a", "__name:=mybag"])
            else:
                if isNodeAlive == True:
                    p=subprocess.Popen(["rosnode", "kill", "/mybag"])                  
            r.sleep()

if __name__ == '__main__':
    a = auto_rosbag()
    a.loop()
