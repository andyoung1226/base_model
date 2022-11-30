#!/usr/bin/python
import time, serial
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int16MultiArray, Int16

wheel_separation = 0.54

manual_chk = 0
obstacle_chk = 0

sturn_sw = False
init_nav = False
init_dist = 0.0
nav_count = 0
pps_sw = False

current_goal = PoseStamped()

cc_goal_x = 0.0
cc_goal_y = 0.0
cc_pose_x = 0.0
cc_pose_y = 0.0
cc_pose_yaw = 0.0

l_forward = False
l_backward = False
r_forward = False
r_backward = False

def Callback(data):
    global sturn_sw, init_nav, nav_count, l_forward, l_backward, r_forward, r_backward
        
    if manual_chk:
        print('manual mode')

    else:
        ts = 100
        
        cc_x = cc_goal_x - cc_pose_x
        cc_y = cc_goal_y - cc_pose_y
        cc_distance = (cc_x**2 + cc_y**2)**0.5
        cc_yawref = np.arctan2(cc_y, cc_x)
        cc_yawerror = cc_yawref - cc_pose_yaw 
        ###

        linear_x = data.linear.x
        angular_z = data.angular.z 

        wheel_speed_left = (linear_x - angular_z * wheel_separation / 2) * 3.6
        wheel_speed_right = (linear_x + angular_z * wheel_separation / 2) * 3.6

        if wheel_speed_left > 0:
            ll = 0x43
        elif wheel_speed_left < 0:
            ll = 0x57
        else:
            ll = 0x53
        if wheel_speed_right > 0:
            rr = 0x43
        elif wheel_speed_right < 0:
            rr = 0x57
        else:
            rr = 0x53

        left_v = abs(int(wheel_speed_left * 10)) + 0x21
        right_v = abs(int(wheel_speed_right * 10)) + 0x21
        
        #left_v = int(left_v *1.3)
        #right_v = int(right_v *1.3)

        #print('dis',cc_distance)
        #print('yaw',cc_yawerror)
        #print('sturn_sw',sturn_sw)
   
        ###
        #sturn_sw = False
        ###

        if sturn_sw == True:# and cc_distance > 1.5:
            #print('dis = ', cc_distance)
            #print('yaw = ', cc_yawerror)
            #print('left_v = ', left_v, ll)
            #print('right_v = ', right_v, rr)
            
            if (abs(cc_yawerror) <= 3.14 and cc_yawerror >= 0) or (abs(cc_yawerror) > 3.14 and cc_yawerror < 0):
                rr = 67
                right_v = 40 #45
                ll = 87
                left_v = right_v
            else:
                ll = 67
                left_v = 40 #45
                rr = 87
                right_v = left_v

            if abs(cc_yawerror) < 0.5 or abs(cc_yawerror) > 5.78:
            #if abs(cc_yawerror) < 0.7 or abs(cc_yawerror) > 5.58:
            #if abs(cc_yawerror) < 0.9 or abs(cc_yawerror) > 5.38:
            #if abs(cc_yawerror) < 1.0 or abs(cc_yawerror) > 5.28:
                sturn_sw = False
                left_v = 33
                right_v = 33
                print('sturn end')
        
        ###
            
        if sturn_sw == True:
            turn_status.publish(1)
        else:
            turn_status.publish(0)

        ###

        if obstacle_chk:
            while True:
                print('obstacle detected !!')
                
                left_v = int(cr_left_v*0.9)
                right_v = int(cr_right_v*0.9)

                ll = cr_ll
                rr = cr_rr

                if left_v < 35:
                    left_v = 33
                if right_v < 35:
                    right_v = 33
                
                if left_v == 33:
                    ll = 0x53
                if right_v == 33:
                    rr = 0x53

                brk = 79
                msg = Int16MultiArray()
                msg.data = [ll,left_v,rr,right_v,brk]
                uart_sub.publish(msg)

                time.sleep(0.1)
            
                if obstacle_chk == 0 or sturn_sw == True:
                    goal_pub.publish(current_goal)
                    sturn_sw = False
                    #init_nav = True
                    #nav_count = 0
                    break
            
        if sturn_sw == False and init_nav == True:
            nav_count += 1
            ff = 0.5 + float(nav_count)/ts
            print('nav_count',nav_count)
            print('ff',ff)
            if ff >= 1:
                init_nav = False
                nav_count = 0

            left_v = int(left_v*ff)
            right_v = int(right_v*ff)
            if left_v < 33:
                left_v = 33
            if right_v < 33:
                right_v = 33
        ###

        if ll == 67:
            l_forward = True
        elif ll == 87:
            l_backward = True
        if rr == 67:
            r_forward = True
        elif rr == 87:
            r_backward = True

        if l_backward == True and ll == 67:
            l_backward = False
            left_v = 33
        elif l_forward == True and ll == 87:
            l_forward = False
            left_v = 33
        if r_backward == True and rr == 67:
            r_backward = False
            right_v = 33
        elif r_forward == True and rr == 87:
            r_forward = False
            right_v = 33

        if left_v == 33:
            ll = 0x53
        if right_v == 33:
            rr = 0x53
            
        brk = 79

        msg = Int16MultiArray()
        msg.data = [ll,left_v,rr,right_v,brk]
        uart_sub.publish(msg)

def UartCallback(data):
    global manual_chk, cr_ll, cr_left_v, cr_rr, cr_right_v
    if data.data[1] == 65:
        manual_chk = 0
    else:
        manual_chk = 1

    cr_ll = data.data[2]
    cr_left_v = data.data[3]
    cr_rr = data.data[4]
    cr_right_v = data.data[5]

def GoalCallback(data):
    global cc_goal_x, cc_goal_y, sturn_sw, init_dist, goal_cnt, init_nav, nav_count, current_goal
    print('goal_CB...')
    cc_goal_x = data.pose.position.x
    cc_goal_y = data.pose.position.y
    init_dist = ((cc_goal_x-cc_pose_x)**2 + (cc_goal_y-cc_pose_y)**2)**0.5

    if pps_sw == True:
        sturn_sw = False
        init_nav = False
    else:
        sturn_sw = True
        init_nav = True
        nav_count = 0

    current_goal = data

def PoseCallback(data):
    global cc_pose_x, cc_pose_y, cc_pose_yaw
    cc_pose_x = data.pose.pose.position.x
    cc_pose_y = data.pose.pose.position.y

    pose_quat = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(pose_quat)
    cc_pose_yaw = yaw

def ObsCallback(data):
    global obstacle_chk, init_nav, nav_count
    obstacle_chk = data.data
    if obstacle_chk:
        init_nav = True
        nav_count = 0

def PpsCallback(data):
    global pps_sw
    if data.data == 1 or data.data == 2:
        pps_sw = True
    elif data.data == 0 or data.data == 3:
        pps_sw = False

if __name__=="__main__":
    rospy.init_node("uart_sub")
    uart_sub = rospy.Publisher("uart_sub", Int16MultiArray, queue_size=1)
    turn_status = rospy.Publisher("turn_status", Int16, queue_size=1)
    goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    while not rospy.is_shutdown():
        rospy.Subscriber('cmd_vel', Twist, Callback)
        rospy.Subscriber("uart_mode_pub", Int16MultiArray, UartCallback)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, GoalCallback) 
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
        rospy.Subscriber('laser_obstacle', Int16, ObsCallback)
        rospy.Subscriber("pp_status", Int16, PpsCallback)
        rospy.sleep(100)
