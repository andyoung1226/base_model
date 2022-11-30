#!/usr/bin/python
import time, serial
import rospy
from std_msgs.msg import UInt8MultiArray, Int16MultiArray, Int16

def UartCallback(data):
    mode_data = []
    for val in data.data:
        mode_data.append(ord(val))

    lw = mode_data[2]
    lws = mode_data[3]
    rw = mode_data[4]
    rws = mode_data[5]

    mode_lws = rws
    mode_rws = lws
    
    if lw == 67:
        mode_rw = 87
    elif lw == 87:
        mode_rw = 67
    elif lw == 83:
        mode_rw = lw
    
    if rw == 67:
        mode_lw = 87
    elif rw == 87:
        mode_lw = 67
    elif rw == 83:
        mode_lw = rw
    
    if mode_msg == 1:
        mode_data[2] = mode_lw
        mode_data[3] = mode_lws
        mode_data[4] = mode_rw
        mode_data[5] = mode_rws
    else:
        pass

    msg = Int16MultiArray()
    msg.data = mode_data
    print(msg.data)
    uart_mode_pub.publish(msg)

def ModeCallback(data):
    global mode_msg
    mode_msg = data.data

if __name__=="__main__":
    rospy.init_node("uart_mode_pub")
    uart_mode_pub = rospy.Publisher("uart_mode_pub", Int16MultiArray, queue_size=1)
    mode_msg = 0
    while not rospy.is_shutdown():
        rospy.Subscriber("uart_pub", UInt8MultiArray, UartCallback)
        rospy.Subscriber("mode", Int16, ModeCallback)
        rospy.sleep(100)

