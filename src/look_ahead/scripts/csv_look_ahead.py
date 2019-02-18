#! /usr/bin/python
# coding:utf-8

import rospy
import numpy as np
import csv
import time

import load_waypoint

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

# ros custom message
from look_ahead.msg import AJK_value
from look_ahead.msg import Auto_Log

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply

look_ahead_dist = 1.3  # look-ahead distance [meter]
SPACING = 0.6       # distance between lines 
x_tolerance = 0.1  # [meter]
yaw_tolerance = 40.0 # [Degree]

# translation value
FORWARD_CONST = 1
BACKWARD_CONST = -1

# AJK
TRANSLATION_NEUTRAL = 512     # neutral value
STEERING_NEUTRAL = 512        # neutral value
RIGHT_PIVOT = 332
LEFT_PIVOT = 692
FB_OPTIMUM = 220
LR_OPTIMUM = 60

# gain
KP = 0.7
KD = 1.0

# frequency [Hz]
frequency = 10

class look_ahead():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_goal = []
        self.x = 0
        self.y = 0
        self.q = np.empty(4)
        self.yaw = np.pi/2
        self.pre_steering_ang = 0

        rospy.init_node('look_ahead_following')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function, receive /odom mesage
        #rospy.Subscriber('/gnss_odom', Odometry, self.odom_callback, queue_size = 1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_callback)
        self.ajk_pub = rospy.Publisher('/ajk_auto', AJK_value, queue_size = 1)
        self.ajk_value = AJK_value()
        self.auto_log_pub = rospy.Publisher('/auto_log', Auto_Log, queue_size = 1)
        self.auto_log = Auto_Log()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # vehicle's quaternion data in /odom (odometry of ROS message)
        self.q[0] = msg.pose.pose.orientation.x
        self.q[1] = msg.pose.pose.orientation.y
        self.q[2] = msg.pose.pose.orientation.z
        self.q[3] = msg.pose.pose.orientation.w

    # truth position of simulator
    def truth_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "sim_ajk":
                self.x = msg.pose[i].position.x
                self.y = msg.pose[i].position.y
                # vehicle's quaternion data in /odom (odometry of ROS message)
                self.q[0] = msg.pose[i].orientation.x
                self.q[1] = msg.pose[i].orientation.y
                self.q[2] = msg.pose[i].orientation.z
                self.q[3] = msg.pose[i].orientation.w

    def shutdown(self):
        print "shutdown"

    def loop(self):
        seq = 1
        while not rospy.is_shutdown():
            # if a specific variable is exists, the proceeds 
            try:
                own_x = self.x
                own_y = self.y
                front_q = self.q
            except AttributeError:
                continue

            # waypoint with xy coordinate origin adjust
            if seq == 0:
                wp_x_adj = self.waypoint_x[seq] - own_x
                wp_y_adj = self.waypoint_y[seq] - own_y
                own_x_adj = 0
                own_y_adj = 0
            else:
                wp_x_adj = self.waypoint_x[seq] - self.waypoint_x[seq-1]
                wp_y_adj = self.waypoint_y[seq] - self.waypoint_y[seq-1]
                own_x_adj = own_x - self.waypoint_x[seq-1]
                own_y_adj = own_y - self.waypoint_y[seq-1]

            # coordinate transformation of waypoint
            tf_angle = np.arctan2(wp_y_adj, wp_x_adj)
            wp_x_tf = wp_x_adj*np.cos(-tf_angle) - wp_y_adj*np.sin(-tf_angle)
            wp_y_tf = wp_x_adj*np.sin(-tf_angle) + wp_y_adj*np.cos(-tf_angle)

            # coordinate transformation of own position
            own_x_tf = own_x_adj*np.cos(-tf_angle) - own_y_adj*np.sin(-tf_angle)
            own_y_tf = own_x_adj*np.sin(-tf_angle) + own_y_adj*np.cos(-tf_angle)


            # coordinate transformation of own position
            tf_q = quaternion_from_euler(0, 0, tf_angle)
            front_q_tf = quaternion_multiply((front_q[0], front_q[1], front_q[2], front_q[3]), 
                                             (   tf_q[0],    tf_q[1],    tf_q[2],   -tf_q[3]))

            # inverted
            rear_q_tf = np.empty(4)
            rear_q_tf[0] = front_q_tf[0]
            rear_q_tf[1] = front_q_tf[1]
            rear_q_tf[2] = front_q_tf[3]
            rear_q_tf[3] = -front_q_tf[2]

            # calculate the distance of target line
            """u = np.array([wp_x_tf, wp_y_tf])
            v = np.array([own_x_tf, own_y_tf])
            d = np.cross(u, v) / np.linalg.norm(u)"""

            # calculate the target-angle(bearing) using look-ahead distance
            bearing = np.arctan2(-own_y_tf, look_ahead_dist)
            bearing_q = quaternion_from_euler(0, 0, bearing)

            # calculate the minimal yaw error, and decide the forward or backward
            front_steering_q = quaternion_multiply(( bearing_q[0],  bearing_q[1],  bearing_q[2],  bearing_q[3]),
                                                   (front_q_tf[0], front_q_tf[1], front_q_tf[2], -front_q_tf[3]))
            rear_steering_q = quaternion_multiply((bearing_q[0], bearing_q[1], bearing_q[2], bearing_q[3]),
                                                  (rear_q_tf[0], rear_q_tf[1], rear_q_tf[2], -rear_q_tf[3]))

            front_steering_ang = euler_from_quaternion(front_steering_q)[2]/np.pi *180
            rear_steering_ang = euler_from_quaternion(rear_steering_q)[2]/np.pi *180

            if abs(front_steering_ang) >= abs(rear_steering_ang):
                steering_ang = rear_steering_ang
                translation = BACKWARD_CONST
            elif abs(front_steering_ang) < abs(rear_steering_ang):
                steering_ang = front_steering_ang
                translation = FORWARD_CONST

            #print steering_ang/np.pi*180.0

            # calculate the steering_value
            p = KP *steering_ang/180
            d = KD *self.pre_steering_ang/180
            pd_value = p + d
            self.pre_steering_ang = steering_ang

            ajk_steering = STEERING_NEUTRAL +LR_OPTIMUM*pd_value
            ajk_translation = TRANSLATION_NEUTRAL +FB_OPTIMUM*translation

            # If the yaw error is large, pivot turn.
            if abs(steering_ang) > yaw_tolerance:
                if steering_ang >= 0:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = LEFT_PIVOT
                else:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = RIGHT_PIVOT                    
            else:
                self.ajk_value.stamp = rospy.Time.now()
                self.ajk_value.translation = ajk_translation
                self.ajk_value.steering = ajk_steering
            self.ajk_pub.publish(self.ajk_value)

            #print wp_x_adj, wp_y_adj, tf_angle/np.pi*180

            # publish autonomous log
            self.auto_log.stamp = rospy.Time.now()
            self.auto_log.waypoint_seq = seq
            self.auto_log.waypoint_x = self.waypoint_x[seq]
            self.auto_log.waypoint_y = self.waypoint_y[seq]
            self.auto_log.tf_waypoint_x = wp_x_tf
            self.auto_log.tf_waypoint_y = wp_y_tf
            self.auto_log.tf_own_x = own_x_tf
            self.auto_log.tf_own_y = own_y_tf
            self.auto_log.cross_track_error = -own_y_tf
            self.auto_log.p = p
            self.auto_log.d = d
            self.auto_log.steering_ang = steering_ang
            self.auto_log.translation = self.ajk_value.translation
            self.auto_log.steering = self.ajk_value.steering
            self.auto_log_pub.publish(self.auto_log)

            """print "sequence:", seq
            print "transform_wx:", wp_x_tf, "transform_wy:", wp_y_tf
            print "transform_own_x:", own_x_tf, "transform_own_y:", own_y_tf
            print "cross_track_error:", d
            print front_steering_ang, rear_steering_ang
            print steering_ang, pd_value
            print self.ajk_value.translation, self.ajk_value.steering"""

            # when reaching the look-ahead distance, read the next waypoint.
            if (wp_x_tf - own_x_tf) < x_tolerance:
                pre_wp_x = self.waypoint_x[seq]
                pre_wp_y = self.waypoint_y[seq]
                seq = seq + 1
                try:
                    a = np.array([pre_wp_x, pre_wp_y])
                    b = np.array([self.waypoint_x[seq], self.waypoint_y[seq]])

                    if np.linalg.norm(a-b) < SPACING:
                        seq = seq + 1
                except IndexError:
                    pass

            if seq >= len(self.waypoint_x):
                self.ajk_value.stamp = rospy.Time.now()
                self.ajk_value.translation = TRANSLATION_NEUTRAL
                self.ajk_value.steering = STEERING_NEUTRAL
                self.ajk_pub.publish(self.ajk_value)
                print "mission_end"
                break
            #print
            time.sleep(1/frequency)

    # load waypoint list
    def load_waypoint(self):
        self.waypoint_x, self.waypoint_y = load_waypoint.load_csv()
        #print self.waypoint_x, self.waypoint_y
    
if __name__ == '__main__':
    l = look_ahead()
    l.load_waypoint()
    l.loop()
