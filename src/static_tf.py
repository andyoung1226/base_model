#!/usr/bin/python
import rospy
import tf
from std_msgs.msg import Int16

def ModeCallback(data):
    global mode_msg
    mode_msg = data.data

if __name__=="__main__":
    rospy.init_node("static_tf")
    bc = tf.TransformBroadcaster()
    mode_msg = 0
    while not rospy.is_shutdown():
        rospy.Subscriber("mode", Int16, ModeCallback)

        if mode_msg == 1:
            bc.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"laser","base_link")
            bc.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"vectornav","base_link")
        else:
            bc.sendTransform((0,0,0),(0,0,1,0),rospy.Time.now(),"laser","base_link")
            bc.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"vectornav","base_link")

        rospy.sleep(0.01)

