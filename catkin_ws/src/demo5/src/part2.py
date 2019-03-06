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
from ros_numpy import numpify

global current_marker_pose
global current_pose
current_marker_pose = None
current_pose = None

def odom_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def img_callback(msg):
    return

def marker_cb(msg):
    global current_marker_pose
    # print msg.markers
    if len(msg.markers):
        current_marker_pose = msg.markers[0].pose.pose
        # print current_marker_pose

rospy.init_node('part2')
img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback)
# img_pub = rospy.Publisher('transformed_img', Image, queue_size=1)
odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_cb)
cmd_vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
# client.wait_for_server()



class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['servo'])
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0.3
        
    def execute(self, data):
        while not rospy.is_shutdown():
            continue
            # cmd_vel_pub.publish(self.twist)
            # if current_marker_pose is not None:
            #     return 'servo'

class VisualServo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate'])
        self.twist = Twist()
        print "init"
    def execute(self, data):
        global current_marker_pose
        # rospy.loginfo("VisualServo")
        print "start"
        while not rospy.is_shutdown():
            # print current_marker_pose
            if (current_marker_pose is not None) and (current_marker_pose.position.x > 1.3):
                print current_marker_pose.position.x
                self.twist.linear.x = 0.1
                self.twist.angular.z = current_marker_pose.position.y * 2
                cmd_vel_pub.publish(self.twist)
        return 'navigate'

class NavigateGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])
        self.twist = Twist()
        print "init"
    def execute(self, data):
        global current_marker_pose
        global current_pose
        # rospy.loginfo("VisualServo")
        print "navigate"
        while not rospy.is_shutdown():

            # print current_marker_pose
            if (current_marker_pose is not None) and (current_marker_pose.position.x > 1.3):
                # pose = numpify(pose_msg) 
                # pose.matmul(...)
                # __, __, angles, translate, __ = decompose_matrix(pose)
                '''
                (trans1, rot1) = tf.lookupTransform(current_marker_pose, current_pose, t)
                trans1_mat = tf.transformations.translation_matrix(trans1)
                rot1_mat   = tf.transformations.quaternion_matrix(rot1)
                mat1 = numpy.dot(trans1_mat, rot1_mat)

                (trans2, rot2) = tf.lookupTransform(l4, l3, t)
                trans2_mat = tf.transformations.translation_matrix(trans2)
                rot2_mat    = tf.transformations.quaternion_matrix(rot2)
                mat2 = numpy.dot(trans2_mat, rot2_mat)

                mat3 = numpy.dot(mat1, mat2)
                trans3 = tf.transformations.translation_from_matrix(mat3)
                rot3 = tf.transformations.quaternion_from_matrix(mat3)
                '''
            #     print current_marker_pose.position.x
            #     self.twist.linear.x = 0.1
            #     self.twist.angular.z = current_marker_pose.position.y * 2
            #     cmd_vel_pub.publish(self.twist)
        return 'start'
'''
class NavigateGoal(smach.state):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])
        self.twist = Twist()
    def execute(self, data):
        global current_marker_pose
        global current_pose
        print "navigate goal"
        # goal_pose = MoveBaseGoal()
        # goal_pose.target_pose.header.frame_id = 'odom'
        # goal_pose.target_pose.pose = current_marker_pose
        # client.send_goal(goal)
        # client.wait_for_result()

        # goal_pose.target_pose.pose.position.x = pose[0][0]
        # goal_pose.target_pose.pose.position.y = pose[0][1]
        # goal_pose.target_pose.pose.position.z = pose[0][2]
        # goal_pose.target_pose.pose.orientation.x = pose[1][0]
        # goal_pose.target_pose.pose.orientation.y = pose[1][1]
        # goal_pose.target_pose.pose.orientation.z = pose[1][2]
        # goal_pose.target_pose.pose.orientation.w = pose[1][3]

        return 'start'
'''
'''
class NavigateStart(smach.state):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate'])
        self.twist = Twist()
    def execute(self, data):
        
        return 'start'
'''

# follower = Follower()
sm = smach.StateMachine(outcomes=['finish'])
with sm:
    # Add states to the container
    smach.StateMachine.add('TURN', Turn(), 
                 transitions={'servo':'SERVO'})
    smach.StateMachine.add('SERVO', VisualServo(), 
                 transitions={'navigate':'GOAL'})
    smach.StateMachine.add('GOAL', NavigateGoal(), 
                 transitions={'start':'finish'})
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