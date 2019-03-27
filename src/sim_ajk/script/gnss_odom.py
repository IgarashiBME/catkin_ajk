#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# using GNSS UTM cordinate date

import rospy
import numpy as np
import time
import pyproj

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from ubx_analyzer.msg import NavPVT
from ubx_analyzer.msg import UTMHP
from ubx_analyzer.msg import RELPOSNED

from tf.transformations import euler_from_quaternion

easting_const = 368000   #[meter]
northing_const = 3955746 #[meter]

# constant
SATELLITES = 10
RTK_FIXED = 2

class gnss_odom():
    def __init__(self):
        rospy.init_node('gnss_imu_sim_ajk_node')
        rospy.on_shutdown(self.shutdown)
        self.utm = Odometry()
        self.pub_utm = rospy.Publisher('/gnss_odom', Odometry, queue_size = 1)
        self.navpvt = NavPVT()
        self.pub_navpvt = rospy.Publisher('/navpvt', NavPVT, queue_size = 1)
        self.relposned = RELPOSNED()
        self.pub_utmhp = rospy.Publisher('/utm_hp', UTMHP, queue_size = 1)
        self.utmhp = UTMHP()
        self.pub_relposned = rospy.Publisher('/relposned', RELPOSNED, queue_size = 1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback, queue_size=1) # ROS callback function

    def callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "sim_ajk":
                self.x = msg.pose[i].position.x
                self.y = msg.pose[i].position.y
                self.z = msg.pose[i].position.z
                self.qx = msg.pose[i].orientation.x
                self.qy = msg.pose[i].orientation.y
                self.qz = msg.pose[i].orientation.z
                self.qw = msg.pose[i].orientation.w

    def navpvt_publisher(self, x, y):
        src_proj = pyproj.Proj(proj='utm', zone=54, ellps='WGS84')
        dst_proj = pyproj.Proj(proj='longlat', ellps='WGS84', datum='WGS84')
        lon, lat = pyproj.transform(src_proj, dst_proj, x, y)

        self.navpvt.lat = lat
        self.navpvt.lon = lon
        self.navpvt.numSV = SATELLITES
        self.navpvt.fix_status = RTK_FIXED
        self.pub_navpvt.publish(self.navpvt)

    def utmhp_publisher(self, x, y):
        src_proj = pyproj.Proj(proj='utm', zone=54, ellps='WGS84')
        dst_proj = pyproj.Proj(proj='longlat', ellps='WGS84', datum='WGS84')
        lon, lat = pyproj.transform(src_proj, dst_proj, x, y)

        self.utmhp.latHp = lat
        self.utmhp.lonHp = lon
        self.utmhp.numSV = SATELLITES
        self.utmhp.fix_status = RTK_FIXED
        self.pub_utmhp.publish(self.utmhp)

    def relposned_publisher(self, qx, qy, qz, qw):
        heading = euler_from_quaternion((qx,qy,qz,qw))[2]
        QGC_heading = -heading +np.pi/2
        if QGC_heading > np.pi:
            QGC_heading = QGC_heading - np.pi*2

        self.relposned.QGC_heading = QGC_heading
        self.pub_relposned.publish(self.relposned)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                x = self.x
                y = self.y
                z = self.z
                qx = self.qx
                qy = self.qy
                qz = self.qz
                qw = self.qw
            except AttributeError:
                continue

            self.utm.header.stamp = rospy.Time.now()
            self.utm.pose.pose.position.x = x + easting_const
            self.utm.pose.pose.position.y = y + northing_const
            self.utm.pose.pose.position.z = z

            self.utm.pose.pose.orientation.x = qx
            self.utm.pose.pose.orientation.y = qy
            self.utm.pose.pose.orientation.z = qz
            self.utm.pose.pose.orientation.w = qw
            self.pub_utm.publish(self.utm)

            self.navpvt_publisher(x + easting_const, y + northing_const)
            self.utmhp_publisher(x + easting_const, y + northing_const)
            self.relposned_publisher(qx, qy, qz, qw)

            time.sleep(0.2)

    def shutdown(self):
        rospy.loginfo("simulate_gnss_odom_node was terminated")

if __name__ == '__main__':
    g = gnss_odom()
    g.loop()
