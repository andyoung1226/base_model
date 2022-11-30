#!/usr/bin/python
import rospy
import tf
from math import pi
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

def ModeCallback(data):
    global forward_mode, backward_mode, reverse_sw
    
    if data.data == 1 and backward_mode == False:
        forward_mode = False
        backward_mode = True
        reverse_sw = True
    elif data.data == 0 and forward_mode == False:
        forward_mode = True
        backward_mode = False
        reverse_sw = True

def PoseCallback(data):
    global reverse_sw

    if reverse_sw == True:
        msg = PoseWithCovarianceStamped()
        msg.header = data.header
        msg.header.frame_id = "map"
        msg.pose = data.pose

        print(data.pose.pose.orientation.z)
        print(data.pose.pose.orientation.w)
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        if data.pose.pose.orientation.z >= 0:
            msg.pose.pose.orientation.z = -w
            msg.pose.pose.orientation.w = z
        else:
            msg.pose.pose.orientation.z = w
            msg.pose.pose.orientation.w = -z

        print('------------')
        print(msg.pose.pose.orientation.z)
        print(msg.pose.pose.orientation.w)

        init_pose.publish(msg)

        reverse_sw = False
    else:
        pass

if __name__=="__main__":
    rospy.init_node("init_pose_pub")
    init_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
    forward_mode = True
    backward_mode = False
    reverse_sw = False
    while not rospy.is_shutdown():
        rospy.Subscriber("mode", Int16, ModeCallback)
        rospy.Subscriber("amcl_odom", Odometry, PoseCallback)
        rospy.sleep(100)

        

