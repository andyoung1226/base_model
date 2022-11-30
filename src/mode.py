#!/usr/bin/python
import rospy
import threading
import sys, select, termios, tty
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, IntParameter

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
	
def obstacle_mode():
    key = getKey(0.1)

    srv_call = rospy.ServiceProxy('/move_base/TebLocalPlannerROS/set_parameters', Reconfigure)
    config = Config()
    param = IntParameter()

    if key == "t":
        print("Transportation mode")
        param.name = 'min_samples'
        param.value = 8
        config.ints = [param]
        srv_call(config)

    elif key == "n":
        print("Non-transportation mode")
        param.name = 'min_samples'
        param.value = 10
        config.ints = [param]
        srv_call(config)

    rospy.sleep(0.01)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("change_mode")
    while not rospy.is_shutdown():
        obstacle_mode()            




