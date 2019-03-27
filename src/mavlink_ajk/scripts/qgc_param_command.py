#! /usr/bin/env python

import rospy
import time
import os

class ajk_shutdown():
    def __init__(self):
        rospy.init_node('ajk_shutdown_node')
        rospy.on_shutdown(self.shutdown)

    def loop(self):
        while not rospy.is_shutdown():
            shutdown_value = rospy.get_param("/mavlink_ajk/shutdown", 0.0)

            if shutdown_value != 0.0:
                os.system('echo "ubuntu" | sudo -S shutdown -h now')
            time.sleep(5)

    def shutdown(self):
        rospy.loginfo("ajk_shutdown_node was terminated")

if __name__ == '__main__':
    a = ajk_shutdown()
    a.loop()
