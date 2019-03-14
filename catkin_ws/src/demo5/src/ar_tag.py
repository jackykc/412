#!/usr/bin/env python
# Retrieved from https://github.com/osrf/rosbook/blob/master/code/navigation/src/patrol.py added additional goals
import rospy
import actionlib

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import numpy as np
from ros_numpy import numpify, geometry

from tf.transformations import decompose_matrix, euler_from_quaternion


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan, Joy

global client, waypoints, waypoint_id, current_marker_pose, current_amcl_pose
global start_detect
start_detect = False
waypoint_id = 0

current_marker_pose = None

waypoints = [  # <1>
    [(4.2810,  -0.118, 0.0), (0.0, 0.0, -0.4471,  0.89448)], # 0
    [(3.745, -0.521, 0.0), (0.0, 0.0, -0.505, 0.862)], # 1
    [(3.037, -0.8509, 0.0), (0.0, 0.0, -0.482, 0.8756)], # 2 # parking spot 3
    [(2.25, -1.20, 0.0), (0.0, 0.0, -0.4735, 0.9058)], # 3


    [(1.492, -1.613, 0.0), (0.0, 0.0, -0.600, 0.7999)], # 4
    [(2.9047, 0.4278, 0.0), (0.0, 0.0, 0.8565, 0.516)], # 5
    [(2.0882, 0.1414, 0.0), (0.0, 0.0, 0.88392, 0.4676)], # 6
    [(1.2, -0.86, 0.0), (0.0, 0.0, -0.9119, 0.410)], # 7

    [(3.539, 1.4547, 0.0), (0.0, 0.0,  0.890, 0.44798)], # 8 end of course   

    [(2.866, -0.227, 0.0), (0.0, 0.0,  -0.9901, 0.139)], # 9 middle
    [(0.1780, -0.0476, 0.0), (0.0, 0.0,  0.2488, 0.96855)], # 10 start
    [(0.6432, 0.396, 0.0), (0.0, 0.0,  -0.0287, 0.99958)], # 11 fork
    [(1.2163, 0.14463, 0.0), (0.0, 0.0,  -0.2283, 0.9735)], # 12 end of line
]

def marker_cb(msg):
    if len(msg.markers):
        # print msg.markers

        for marker in msg.markers:
            global current_marker_pose
            current_marker_pose = marker.pose.pose


def joy_callback(msg):
    global waypoints, client
    if msg.buttons[0]:
        goal = goal_pose(waypoints[0])
        client.send_goal(goal)
        client.wait_for_result()
    elif msg.buttons[1]:
        goal = goal_pose(waypoints[1])
        client.send_goal(goal)
        client.wait_for_result()
    elif msg.buttons[2]:
        goal = goal_pose(waypoints[2])
        client.send_goal(goal)
        client.wait_for_result()
    elif msg.buttons[3]:
        goal = goal_pose(waypoints[3])
        client.send_goal(goal)
        client.wait_for_result()


def goal_pose(pose):  # <2>
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

def initial_pose(pose):
    goal_pose = PoseWithCovarianceStamped()
    goal_pose.pose.pose.position.x = pose[0][0]
    goal_pose.pose.pose.position.y = pose[0][1]
    goal_pose.pose.pose.position.z = pose[0][2]
    goal_pose.pose.pose.orientation.x = pose[1][0]
    goal_pose.pose.pose.orientation.y = pose[1][1]
    goal_pose.pose.pose.orientation.z = pose[1][2]
    goal_pose.pose.pose.orientation.w = pose[1][3]

    return goal_pose

def amcl_pose_callback(msg):
    global current_amcl_pose
    # global initial_pose
    current_amcl_pose = msg.pose.pose
    # if initial_pose is None:
    #     initial_pose = current_pose

if __name__ == '__main__':
    global client, current_marker_pose, current_amcl_pose, start_detect
    rospy.init_node('patrol')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()

    marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_cb)
    cmd_vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    # rospy.Subscriber("joy", Joy, joy_callback)

    # initial_pose = initial_pose(waypoints[10])
    # initial_pose_pub.publish(initial_pose)
    goal = goal_pose(waypoints[11])
    client.send_goal(goal)
    client.wait_for_result()
    goal = goal_pose(waypoints[12])
    client.send_goal(goal)
    client.wait_for_result()

    goal = goal_pose(waypoints[9])
    client.send_goal(goal)
    client.wait_for_result()

    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0.4
        
    while not rospy.is_shutdown():

        # while current_marker_pose is None:
        #     cmd_vel_pub.publish(twist)
        #     continue

        for index, pose in enumerate(waypoints[0:8]):
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
            start_detect = True
            current_marker_pose = None
            wait_time = rospy.Time.now() + rospy.Duration(1)
            while rospy.Time.now()<wait_time:
                continue
            if current_marker_pose is not None:
                print index
                print pose
                print current_marker_pose.position
            start_detect = False
            current_marker_pose = None

        '''
        # got the ar tag
        np_ar_pose = numpify(current_marker_pose)
        np_current_amcl_pose = numpify(current_amcl_pose)
        np_goal_pose = np.matmul(np_current_amcl_pose, np_ar_pose)
        geometry_pose = geometry.numpy_to_pose(np_goal_pose)

        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose = geometry_pose
        goal_pose.target_pose.pose.position.z = 0.0
        goal_pose.target_pose.pose.orientation.x = 0.0
        goal_pose.target_pose.pose.orientation.y = 0.0

        print "AR"
        print current_marker_pose
        print "AMCL"
        print current_amcl_pose
        print "GOAL"
        print goal_pose.target_pose.pose
        
        client.send_goal(goal_pose)
        client.wait_for_result()
        '''
        exit()
        # for pose in waypoints:   # <4>
        #     goal = goal_pose(pose)
        #     print pose
        #     client.send_goal(goal)
        #     client.wait_for_result()
        # break