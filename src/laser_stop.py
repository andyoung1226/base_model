#!/usr/bin/python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int16MultiArray, Float64MultiArray, Bool
from sensor_msgs.msg import LaserScan

def ScanCallback(msg):
  global laser_obs
  
  data = np.where(np.array(msg.ranges) < obs_detection_distance)
  #print(data)

  if np.array(data).size!=0:
    laser_obs = True
    print("Obstacle Detected, Emergency STOP!")
  else:
    laser_obs = False
    print("No Obstacle Detected, Free Driving!")
        
  obs_pub.publish(laser_obs)

if __name__=="__main__":
  rospy.init_node("Laser_stop")
  obs_pub = rospy.Publisher("is_obs", Bool, queue_size=1)
  laser_obs = False
  obs_detection_distance = 1.5

  rospy.Subscriber("scan_filtered", LaserScan, ScanCallback)
  rospy.spin()
