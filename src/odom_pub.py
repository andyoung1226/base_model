#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray

class Odometry_pub(object):
	def __init__(self):
		self.last_time = rospy.Time.now()
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
		#self.odom_broadcaster = tf.TransformBroadcaster()
		#self.odom_sub = rospy.Subscriber("/vectornav/IMU", Imu, self.OdomCallback)
		#self.odom_sub = rospy.Subscriber("imu/data", Imu, self.OdomCallback)
		self.uart_sub = rospy.Subscriber("uart_mode_pub", Int16MultiArray, self.UartCallback)
		
                self.x = 0
		self.y = 0
		self.v = 0
		self.vx = 0
		self.vy = 0
		self.th = 0
		self.vth = 0
                self.init_th = 0

	def UartCallback(self, msg):
		if msg.data[2] == 67:
			lv = float(msg.data[3] - 33.0) / 10.0 
		else:
			lv = -float(msg.data[3] - 33.0) / 10.0	
                if msg.data[4] == 67:		
			rv = float(msg.data[5] - 33.0) / 10.0
		else:
			rv = -float(msg.data[5] - 33.0) / 10.0
		
                self.v = float((lv + rv) / 2 / 3.6) #*1.2 # *1.8 # * 2.2 /2.0
		self.vth = float((rv - lv) /3.6 / 0.54)# *0.7
                self.odom_pub.publish(self.Odometry_msg())
        '''
	def OdomCallback(self, msg):
	#	print("odom")
		ori = msg.orientation
		ori = [ori.x, ori.y, ori.z, ori.w]
		roll, pitch, yaw = tf.transformations.euler_from_quaternion(ori)
                
                if msg.header.seq < 5:
                    self.init_th = yaw
                #print(self.init_th)
                
                #self.th = - (yaw - self.init_th + 1.570796)
                #self.vth = msg.angular_velocity.z
		#self.odom_pub.publish(self.Odometry_msg())
        '''
	def Odometry_msg(self):
		current_time = rospy.Time.now()
		dt = (current_time - self.last_time).to_sec()
		
		#odom_broadcaster = tf.TransformBroadcaster()

		self.th += self.vth * dt
                #print(self.th)

		self.vx = self.v * cos(self.th)#*1.5)
		self.vy = self.v * sin(self.th)#*1.5)
		delta_x = self.vx * dt
		delta_y = self.vy * dt
		self.x += delta_x
		self.y += delta_y
		#print(self.x)


		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)     
               
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
		odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

		#odom_broadcaster.sendTransform(
		#(self.x, self.y, 0.),
		#odom_quat,
		#current_time,
		#"base_link",
		#"odom"
		#)

		self.last_time = current_time

		return odom

if __name__=="__main__":
	rospy.init_node('odom_pub')
	ss = Odometry_pub()
	rospy.spin()

