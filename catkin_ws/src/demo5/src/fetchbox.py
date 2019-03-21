#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from odom_msgs.msg import 
import actionlib
import tf
import cv2, cv_bridge
import numpy as np
import smach
import smach_ros

from tf.transformations import decompose_matrix, euler_from_quaternion
from ros_numpy import numpify, geometry

import copy

global current_marker_pose
global current_pose
global initial_pose
global finished_marker_ids
global current_marker_id
current_marker_id = None
finished_marker_ids = []
current_marker_pose = None
current_pose = None
initial_pose = None

def odom_callback(msg):
    global initial_pose
    global current_pose
    current_pose = msg.pose.pose
    if initial_pose is None:
        initial_pose = current_pose

def img_callback(msg):
    return

def marker_cb(msg):
    global current_marker_pose
    global finished_marker_ids
    global current_marker_id
    # print msg.markers
    if len(msg.markers):
        for marker in msg.markers:
            current_marker_pose = marker.pose.pose
        # print current_marker_pose

rospy.init_node('part2')
odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_cb)
cmd_vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
client.wait_for_server()

class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['servo'])
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0.4
        
    def execute(self, data):
        global finished_marker_ids
        global current_marker_id
        global current_marker_pose
        current_marker_id = None
        current_marker_pose = None
        while not rospy.is_shutdown():
            cmd_vel_pub.publish(self.twist)
            if current_marker_pose is not None:
                return 'servo'

class NavigateGoalB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['push', 'push'])
    def execute(self, data):
        global current_marker_pose
        global current_pose
        # rospy.loginfo("VisualServo")
        print "navigate"
        
        if (current_marker_pose is not None and current_pose is not None):
            np_current_pose = numpify(current_pose)
            np_current_marker_pose = numpify(current_marker_pose)
            np_goal_pose = np.matmul(np_current_pose, np_current_marker_pose)
            temp_goal_pose = geometry.numpy_to_pose(np_goal_pose)
            temp_goal_pose.position.x += 1.05

            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'odom'
            goal_pose.target_pose.pose = temp_goal_pose
            goal_pose.target_pose.pose.position.z = 0.0
            goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
            goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
            goal_pose.target_pose.pose.orientation.z = -1#-1*current_pose.orientation.z
            goal_pose.target_pose.pose.orientation.w = 0#current_pose.orientation.w

            goal_pose1 = copy.deepcopy(goal_pose)
            goal_pose1.target_pose.pose.position.y -= 0.75
            goal_pose1.target_pose.pose.position.x -= 1.05
            
            client.send_goal(goal_pose1)
            client.wait_for_result()
            
            
            # goal_pose2 = copy.deepcopy(goal_pose)
            # goal_pose2.target_pose.pose.position.y = current_pose.position.y - 0.5
            # # goal_pose2.target_pose.pose.position.x = current_pose.position.x 
            # client.send_goal(goal_pose2)
            # client.wait_for_result()
            
            client.send_goal(goal_pose)
            client.wait_for_result()

        return 'push'

class Push(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn'])
        self.twist = Twist()
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0

    global current_pose
    def execute(self, data):
        while not rospy.is_shutdown():
            cmd_vel_pub.publish(self.twist)
            if current_pose and current_pose.position.x < 0:
                return 'turn'
       

        # return 'finish'

# follower = Follower()
sm = smach.StateMachine(outcomes=['finish'])
with sm:
    # Add states to the container
    smach.StateMachine.add('TURN', Turn(), 
                 transitions={'servo':'GOALB'})
    smach.StateMachine.add('GOALB', NavigateGoalB(), 
                 transitions={'push':'PUSH'})
    smach.StateMachine.add('PUSH', Push(), 
                 transitions={"turn": "TURN"})
    # smach.StateMachine.add('TASK1', Task1(), 
    #              transitions={'go':'GO'})
    # smach.StateMachine.add('TASK2', Task2(), 
    #              transitions={'go':'GO'})
    # smach.StateMachine.add('TASK3', Task3(), 
    #              transitions={'go':'GO'})

# Create and start the introspection server
# sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
# sis.start()

outcome = sm.execute()
rospy.spin()
# sis.stop()