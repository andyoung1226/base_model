#!/usr/bin/env python
import rospy
import time
import tf
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int16

main_points = [[-2.787,3.103,-3.115],[1.702,-0.202,0.007]]

node = 0
pose = False
cc_pose_x = 0.0
cc_pose_y = 0.0

def goal_point(point_position):
    goal_point = PoseStamped()
    
    goal_point.header.stamp = rospy.Time.now()
    goal_point.header.frame_id = "map"
    
    goal_point.pose.position.x = point_position[0]
    goal_point.pose.position.y = point_position[1]
    goal_point.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,point_position[2]))

    return goal_point

def GoalCallback(data):
    global node, pose
    
    if data.status.status == 3:
        print('path planning complete')

        print('waiting...')
        time.sleep(5)

        if node == 0:
            node = 1
        elif node == 1:
            node = 0

        print('Node No. :', node)

        point_position = main_points[node]

        cc_x = point_position[0] - cc_pose_x
        cc_y = point_position[1] - cc_pose_y
        dis = (cc_x**2 + cc_y**2)**0.5
        if dis > 1:
            pose = True

        if pose == True:
            goal = goal_point(point_position)
            goal_pub.publish(goal)
            print('main node publish')
            pose = False

def PoseCallback(data):
    global cc_pose_x, cc_pose_y
    cc_pose_x = data.pose.pose.position.x
    cc_pose_y = data.pose.pose.position.y

if __name__=="__main__":
    rospy.init_node("goal_pub")
    goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    while not rospy.is_shutdown():
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, GoalCallback)
        rospy.sleep(100)

