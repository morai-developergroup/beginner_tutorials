#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from pyproj import Proj
from math import pi
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage,EgoVehicleStatus
class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.x, self.y = None, None
        self.is_imu=False
        self.gps_imu=False
        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link1'
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_imu==True and self.gps_imu == True:
                self.convertLL2UTM()
                rate.sleep()

    def navsat_callback(self, gps_msg):
        self.gps_imu=True
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 0.),
                        tf.transformations.quaternion_from_euler(0, 0, 0.),
                        rospy.Time.now(),
                        "base_link",
                        "map")
        
        utm_msg = Float32MultiArray()
        utm_msg.data = [self.x, self.y]
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0
        self.odom_pub.publish(self.odom_msg)
    def convertLL2UTM(self):
    
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0]
        self.y = xy_zone[1]

    def imu_callback(self, data):
        self.is_imu=True
        self.odom_msg.pose.pose.orientation.x = data.orientation.x
        self.odom_msg.pose.pose.orientation.y = data.orientation.y
        self.odom_msg.pose.pose.orientation.z = data.orientation.z
        self.odom_msg.pose.pose.orientation.w = data.orientation.w
        quaternion=(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
        roll,pitch,yaw=euler_from_quaternion(quaternion)
        roll_deg=roll/pi*180
        pitch_deg=pitch/pi*180
        yaw_deg= yaw/pi*180
        self.prev_time=rospy.get_rostime()


if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
