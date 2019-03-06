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
    [(0.2, 2.3, 0.0), (0.0, 0.0, 0.057, 0.818)],
    [(1.9, 1.3, 0.0), (0.0, 0.0, 0.057, 0.818)],
    [(0.35, 0.42, 0.0), (0.0, 0.0, 0.069, 0.99)],
    [(1.78, -0.12, 0.0), (0.0, 0.0, 0.69, 0.73)],
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
    
    rospy.Subscriber("joy", Joy, joy_callback)
    
    while not rospy.is_shutdown():
        continue
        '''
        for pose in waypoints:   # <4>
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
        '''