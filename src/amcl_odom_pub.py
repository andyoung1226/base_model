#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray

class Odometry_pub(object):
	def __init__(self):
		self.last_time = rospy.Time.now()
		self.odom_pub = rospy.Publisher("amcl_odom", Odometry, queue_size=1)
		#self.odom_broadcaster = tf.TransformBroadcaster()
		#self.odom_sub = rospy.Subscriber("/vectornav/IMU", Imu, self.OdomCallback)
		#self.uart_sub = rospy.Subscriber("uart_pub", UInt8MultiArray, self.UartCallback)
		self.amcl_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.AMCLCallback)

                self.x = 0
		self.y = 0
                self.quat_x = 0
                self.quat_y = 0
                self.quat_z = 0
                self.quat_w = 1

#	def UartCallback(self, msg):
#		if ord(msg.data[2]) == 67:
#			lv = (ord(msg.data[3]) - 33) / 10
#		else:
#			lv = -(ord(msg.data[3]) - 33) / 10		
#		if ord(msg.data[4]) == 67:		
#			rv = (ord(msg.data[5]) - 33) / 10
#		else:
#			rv = -(ord(msg.data[5]) - 33) / 10
#		self.v = (lv + rv) / 2
#		print(lv, rv)
#		print(self.v)
#
#	def OdomCallback(self, msg):		
#		ori = msg.orientation
#		ori = [ori.x, ori.y, ori.z, ori.w]
#		roll, pitch, yaw = tf.transformations.euler_from_quaternion(ori)
#		self.th = yaw
#		print(self.th)
#
#		self.odom_pub.publish(self.Odometry_msg())

	def AMCLCallback(self, msg):
                self.x = msg.pose.pose.position.x
                self.y = msg.pose.pose.position.y
                self.quat_x = msg.pose.pose.orientation.x
                self.quat_y = msg.pose.pose.orientation.y
                self.quat_z = msg.pose.pose.orientation.z
                self.quat_w = msg.pose.pose.orientation.w

		#self.odom_pub.publish(self.Odometry_msg())


	def Odometry_msg(self):
            while not rospy.is_shutdown():
                current_time = rospy.Time.now()
		#dt = (current_time - self.last_time).to_sec()
		
		#odom_broadcaster = tf.TransformBroadcaster()
		
		odom_quat = self.quat_x, self.quat_y, self.quat_z, self.quat_w  
              
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(odom_quat)
                print(yaw)

		amcl_odom = Odometry()
		amcl_odom.header.stamp = current_time
		amcl_odom.header.frame_id = "odom"
		amcl_odom.child_frame_id = "base_link"
		amcl_odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
		amcl_odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

		#odom_broadcaster.sendTransform(
		#(self.x, self.y, 0.),
		#odom_quat,
		#current_time,
		#"base_link",
		#"odom"
		#)

		#self.last_time = current_time

                #rospy.init_node('amcl_odom_pub')

                self.odom_pub.publish(amcl_odom)
                rospy.Rate(100).sleep()

if __name__=="__main__":
	rospy.init_node('amcl_odom_pub')
	ss = Odometry_pub()
        ss.Odometry_msg()

