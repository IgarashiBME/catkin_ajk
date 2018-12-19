#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# using GNSS UTM cordinate date

import rospy
import numpy as np
import time

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion

class gnss_dummy():
    def __init__(self):
        rospy.init_node('imu_dummy_sim_ajk_node')
        rospy.on_shutdown(self.shutdown)
        self.imu = Imu()
        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size = 1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback, queue_size=1) # ROS callback function

    def callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "sim_ajk":
                self.qx = msg.pose[i].orientation.x
                self.qy = msg.pose[i].orientation.y
                self.qz = msg.pose[i].orientation.z
                self.qw = msg.pose[i].orientation.w

    def loop(self):
        pre_t = None
        pre_yaw = None
        while not rospy.is_shutdown():
            try:
                self.qx
                self.qy
                self.qz
            except AttributeError:
                continue

            current_t = time.time()
            self.yaw = euler_from_quaternion([self.qx, self.qy, self.qz, self.qw])[2]
            if self.yaw < 0:    # yaw angle, 0~360 degree
                self.yaw = self.yaw + 2*np.pi

            # forward
            if pre_t != None and pre_yaw != None:
                yaw_error_a = self.yaw - pre_yaw
                yaw_error_b = self.yaw - pre_yaw -2*np.pi
                yaw_error_c = self.yaw - pre_yaw +2*np.pi
                yaw_list = [yaw_error_a, yaw_error_b, yaw_error_c]
                yaw_error = yaw_list[np.argmin(np.abs(yaw_list))] # min yaw error is selected

                angular_vel_z = yaw_error/(current_t - pre_t)
                #print angular_vel

                self.imu.header.stamp = rospy.Time.now()
                self.imu.angular_velocity.z = angular_vel_z
                self.imu.orientation.x = self.qx
                self.imu.orientation.y = self.qy
                self.imu.orientation.z = self.qz
                self.imu.orientation.w = self.qw
                self.pub_imu.publish(self.imu)

            pre_t = current_t
            pre_yaw  = self.yaw
            time.sleep(0.01)

    def shutdown(self):
        rospy.loginfo("sub_robot_node was terminated")

if __name__ == '__main__':
    g = gnss_dummy()
    g.loop()
