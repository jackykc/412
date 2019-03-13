#!/usr/bin/env python
# Retrieved from https://github.com/osrf/rosbook/blob/master/code/navigation/src/patrol.py added additional goals
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan, Joy

global client, waypoints

# waypoints = [  # <1>
#     [(3.49, 5.03, 0.0), (0.0, 0.0, 0.57, 0.818)],
#     [(3.4, 2.7, 0.0), (0.0, 0.0, 0.34, 0.93)],
#     [(4.99, 4.52, 0.0), (0.0, 0.0, 0.069, 0.99)],
#     [(4.88, 1.94, 0.0), (0.0, 0.0, -0.74, 0.66)],
# ]

waypoints = [  # <1>
    [(4.2810,  -0.118, 0.0), (0.0, 0.0, -0.4471,  0.89448)],
    [(3.745, -0.521, 0.0), (0.0, 0.0, -0.505, 0.862)],
    [(3.037, -0.8509, 0.0), (0.0, 0.0, -0.482, 0.8756)],
    [(2.25, -1.20, 0.0), (0.0, 0.0, -0.4735, 0.9058)],


    [(1.492, -1.613, 0.0), (0.0, 0.0, -0.600, 0.7999)],
    [(2.9047, 0.4278, 0.0), (0.0, 0.0, 0.8565, 0.516)],
    [(2.0882, 0.1414, 0.0), (0.0, 0.0, 0.88392, 0.4676)],
    [(1.2, -0.86, 0.0), (0.0, 0.0, -0.9119, 0.410)],

    [(3.539, 1.4547, 0.0), (0.0, 0.0,  0.890, 0.44798)], # end of course   

    [(2.866, -0.227, 0.0), (0.0, 0.0,  -0.9901, 0.139)], # middle
    [(0.1780, -0.0476, 0.0), (0.0, 0.0,  0.2488, 0.96855)], # start
    [(0.6432, 0.396, 0.0), (0.0, 0.0,  -0.0287, 0.99958)], # fork
    [(1.2163, 0.14463, 0.0), (0.0, 0.0,  -0.2283, 0.9735)], # end of line
]


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

if __name__ == '__main__':
    global client
    rospy.init_node('patrol')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()
    
    # rospy.Subscriber("joy", Joy, joy_callback)
    
    while not rospy.is_shutdown():
        
        for pose in waypoints:   # <4>
            goal = goal_pose(pose)
            print pose
            client.send_goal(goal)
            client.wait_for_result()
        break