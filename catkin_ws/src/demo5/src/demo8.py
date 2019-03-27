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
global parking_spot_pose
global direction
current_marker_pose = None
current_pose = None
initial_pose = None
parking_spot_pose = None
direction = 1
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
    if len(msg.markers):
        for marker in msg.markers:
            if (msg.markers[0].id == 4) or (msg.markers[0].id == 2):
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
        smach.State.__init__(self, outcomes=['turn_back'])
        self.twist = Twist()
        self.twist.linear.x = 0
        self.angular_speed = 0.4
        self.twist.angular.z = self.angular_speed
        
        
    def execute(self, data):
        global current_marker_pose, current_pose, parking_spot_pose
        current_marker_pose = None

        twist = Twist()
        twist.linear.x = 0.2
        wait_time = rospy.Time.now()+rospy.Duration(5)
        while rospy.Time.now() < wait_time:
            cmd_vel_pub.publish(twist)

        while not rospy.is_shutdown():
            pose = numpify(current_pose) 
            __, __, angles, translate, __ = decompose_matrix(pose)
            heading = angles[2] * 180 / 3.14159
            cmd_vel_pub.publish(self.twist)
            if heading > 90:
                self.twist.angular.z = -1 * self.angular_speed
            elif heading < -90:
                self.twist.angular.z = self.angular_speed

            if current_marker_pose is not None:
                np_current_pose = numpify(current_pose)
                np_current_parking_pose = numpify(current_marker_pose)
                np_goal_pose = np.matmul(np_current_pose, np_current_parking_pose)
                temp_goal_pose = geometry.numpy_to_pose(np_goal_pose)

                parking_spot_pose = temp_goal_pose # parking spot
                current_marker_pose = None
                break
        
        '''
        # pose for side box
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'odom'
        goal_pose.target_pose.pose = parking_spot_pose
        goal_pose.target_pose.pose.position.x -= 0.3
        # goal_pose.target_pose.pose.position.y += direction * 0.75
        goal_pose.target_pose.pose.position.z = 0.0

        goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
        goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
        goal_pose.target_pose.pose.orientation.z = -1#-1*current_pose.orientation.z
        goal_pose.target_pose.pose.orientation.w = 0#current_pose.orientation.w

        client.send_goal(goal_pose)
        client.wait_for_result()
        ''' 
        wait_time = rospy.Time.now()+rospy.Duration(5)
        twist.linear.x *= -1

        while rospy.Time.now() < wait_time:
            cmd_vel_pub.publish(twist)

        return 'turn_back'

class TurnLookBehind(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_a'])
        self.twist = Twist()
        self.twist.linear.x = 0
        self.angular_speed = 0.7
        self.twist.angular.z = self.angular_speed
        
        
    def execute(self, data):
        global current_marker_pose, current_pose
        current_marker_pose = None

        while not rospy.is_shutdown():
            pose = numpify(current_pose) 
            __, __, angles, translate, __ = decompose_matrix(pose)
            heading = angles[2] * 180 / 3.14159 
            cmd_vel_pub.publish(self.twist)
            if heading > 90:
                break
        
        current_marker_pose = None
        while not rospy.is_shutdown():
            pose = numpify(current_pose) 
            __, __, angles, translate, __ = decompose_matrix(pose)
            heading = angles[2] * 180 / 3.14159
            cmd_vel_pub.publish(self.twist)
            if heading > 0 and heading < 90:
                self.twist.angular.z = self.angular_speed
            elif heading < 0 and heading > -90:
                self.twist.angular.z = -1 * self.angular_speed
            
            if current_marker_pose is not None:
                return 'goal_a'

class NavigateGoalA(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['push'])
    def execute(self, data):
        global current_marker_pose, current_pose, direction
        
        if (current_marker_pose is not None and current_pose is not None):
            np_current_pose = numpify(current_pose)
            np_current_marker_pose = numpify(current_marker_pose)
            np_goal_pose = np.matmul(np_current_pose, np_current_marker_pose)
            temp_goal_pose = geometry.numpy_to_pose(np_goal_pose)
            temp_goal_pose.position.x -= 1.05

            # pose for behind box
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'odom'
            goal_pose.target_pose.pose = temp_goal_pose
            goal_pose.target_pose.pose.position.z = 0.0
            goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
            goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
            goal_pose.target_pose.pose.orientation.z = -1#-1*current_pose.orientation.z
            goal_pose.target_pose.pose.orientation.w = 0#current_pose.orientation.w

            # pose for side of box
            goal_pose1 = copy.deepcopy(goal_pose)
            # left and right TODO: change direction depending on parking spot
            if (goal_pose1.target_pose.pose.position.y < parking_spot_pose.position.y):
                direction = -1
            else:
                direction = 1
            goal_pose1.target_pose.pose.position.y += direction * 0.75
            goal_pose1.target_pose.pose.position.x += 0.95#1.05
            # go to side
            client.send_goal(goal_pose1)
            client.wait_for_result()
            # go behind
            client.send_goal(goal_pose)
            client.wait_for_result()

        return 'push'

class PushX(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_b'])
        self.twist = Twist()
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0

    def execute(self, data):
        global current_pose, parking_spot_pose
        temp_goal_pose = parking_spot_pose
        temp_goal_pose.position.x -= 1.05

        # pose for behind box
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'odom'
        goal_pose.target_pose.pose = temp_goal_pose
        goal_pose.target_pose.pose.position.y = current_pose.position.y
        goal_pose.target_pose.pose.position.z = 0.0
        goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
        goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
        goal_pose.target_pose.pose.orientation.z = 1#-1*current_pose.orientation.z
        goal_pose.target_pose.pose.orientation.w = 0#current_pose.orientation.w

        client.send_goal(goal_pose)
        client.wait_for_result()
        
        # cmd_vel_pub.publish(self.twist)
        # if current_pose and current_pose.position.x < 0:
        #     return 'turn'
        return 'goal_b'

        # return 'finish'

class NavigateGoalB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_back'])
    def execute(self, data):
        global current_pose, parking_spot_pose, direction, initial_pose
        
        park_goal_pose = MoveBaseGoal()
        park_goal_pose.target_pose.header.frame_id = 'odom'
        park_goal_pose.target_pose.pose = initial_pose
        # park_goal_pose.target_pose.pose.position.x = goal_pose.target_pose.pose.position.x
        # park_goal_pose.target_pose.pose.position.y = parking_spot_pose.position.y - (direction*0.5)
        park_goal_pose.target_pose.pose.position.z = 0.0

        park_goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
        park_goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
        park_goal_pose.target_pose.pose.orientation.z = -1#-1*current_pose.orientation.z
        park_goal_pose.target_pose.pose.orientation.w = 0#current_pose.orientation.w


        # go behind
        client.send_goal(park_goal_pose)
        client.wait_for_result()

        '''
        # pose for side box
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'odom'
        goal_pose.target_pose.pose = current_pose
        goal_pose.target_pose.pose.position.x += 0.4
        goal_pose.target_pose.pose.position.y += direction * 0.75
        goal_pose.target_pose.pose.position.z = 0.0

        goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
        goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
        goal_pose.target_pose.pose.orientation.z = -1#-1*current_pose.orientation.z
        goal_pose.target_pose.pose.orientation.w = 0#current_pose.orientation.w

        # backup
        twist = Twist()
        twist.linear.x = 0.2
        wait_time = rospy.Time.now()+rospy.Duration(4)
        while rospy.Time.now() < wait_time:
            cmd_vel_pub.publish(twist)
        
        # go to side
        client.send_goal(goal_pose)
        client.wait_for_result()

        # pose side of parking spot
        park_goal_pose = MoveBaseGoal()
        park_goal_pose.target_pose.header.frame_id = 'odom'
        park_goal_pose.target_pose.pose = parking_spot_pose
        park_goal_pose.target_pose.pose.position.x = goal_pose.target_pose.pose.position.x
        park_goal_pose.target_pose.pose.position.y = parking_spot_pose.position.y - (direction*0.5)
        park_goal_pose.target_pose.pose.position.z = 0.0

        park_goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
        park_goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
        park_goal_pose.target_pose.pose.orientation.z = -1#-1*current_pose.orientation.z
        park_goal_pose.target_pose.pose.orientation.w = 0#current_pose.orientation.w


        # go behind
        client.send_goal(park_goal_pose)
        client.wait_for_result()
        '''
        return 'turn_back'

sm = smach.StateMachine(outcomes=['finish'])
with sm:
    # Add states to the container
    smach.StateMachine.add('TURN', Turn(), 
                 transitions={'turn_back':'TURN_LOOK_BEHIND'})
    smach.StateMachine.add('TURN_LOOK_BEHIND', TurnLookBehind(), 
                 transitions={'goal_a':'GOALA'})
    smach.StateMachine.add('GOALA', NavigateGoalA(), 
                 transitions={'push':'PUSHX'})
    smach.StateMachine.add('PUSHX', PushX(), 
                 transitions={"goal_b": "GOALB"})
    smach.StateMachine.add('GOALB', NavigateGoalB(), 
                 transitions={'turn_back':'TURN_LOOK_BEHIND'})
    
outcome = sm.execute()
rospy.spin()