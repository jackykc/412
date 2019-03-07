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
from ros_numpy import numpify, numpy_to_pose

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
    global current_pose
    global initial_pose
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
            if marker.id in finished_marker_ids:
                continue
            if current_marker_id is None:
                 current_marker_id = marker.id
            current_marker_pose = marker.pose.pose
        # print current_marker_pose

rospy.init_node('part2')
img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback)
# img_pub = rospy.Publisher('transformed_img', Image, queue_size=1)
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
        while not rospy.is_shutdown():
            cmd_vel_pub.publish(self.twist)
            if current_marker_pose is not None:
                return 'servo'

class VisualServo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate'])
        self.twist = Twist()
        print "init"
    def execute(self, data):
        global current_marker_pose
        # rospy.loginfo("VisualServo")
        print "start"
        reached = False
        while not rospy.is_shutdown():
            # print current_marker_pose
            if (current_marker_pose is not None) and (current_marker_pose.position.x > 0.8):
                print current_marker_pose.position.x
                self.twist.linear.x = 0.1
                direction = current_marker_pose.position.y > 0
                if direction: 
                    direction = 1
                else:
                    direction = -1
                self.twist.angular.z = direction * 0.6
                cmd_vel_pub.publish(self.twist)
                reached = True
            elif reached:
                return 'navigate'
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
        
        if (current_marker_pose is not None and current_pose is not None):
            np_current_pose = numpify(current_pose)
            np_current_marker_pose = numpify(current_marker_pose)
            np_goal_pose = np.matmul(np_current_pose, np_current_marker_pose)
            temp_goal_pose = numpy_to_pose(np_goal_pose)


            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'odom'
            goal_pose.pose = temp_goal_pose
            goal_pose.target_pose.pose.position.z = 0.0
            
            '''
            goal_pose.target_pose.pose.position.x = np_goal_pose[0][3]
            goal_pose.target_pose.pose.position.y = np_goal_pose[1][3]
            goal_pose.target_pose.pose.position.z = 0.0
            goal_pose.target_pose.pose.orientation.x = 0#current_marker_pose.orientation.x
            goal_pose.target_pose.pose.orientation.y = 0#current_marker_pose.orientation.y
            goal_pose.target_pose.pose.orientation.z = current_pose.orientation.z
            goal_pose.target_pose.pose.orientation.w = current_pose.orientation.w
            '''

            # print np_goal_pose
            print goal_pose
            print current_pose
            global finished_marker_ids
            global current_marker_id
            client.send_goal(goal_pose)
            client.wait_for_result()

        return 'start'

class NavigateStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn', 'finish'])
    def execute(self, data):
        global current_marker_pose
        global current_pose
        global initial_pose
        print "start"
        
        if (current_marker_pose is not None and current_pose is not None):
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'odom'
            goal_pose.target_pose.pose = initial_pose
            global finished_marker_ids
            global current_marker_id
            print finished_marker_ids
            print current_marker_id
            client.send_goal(goal_pose)
            client.wait_for_result()
            finished_marker_ids.append(current_marker_id)
            current_marker_id = None

        return 'turn'


# follower = Follower()
sm = smach.StateMachine(outcomes=['finish'])
with sm:
    # Add states to the container
    smach.StateMachine.add('TURN', Turn(), 
                 transitions={'servo':'SERVO'})
    smach.StateMachine.add('SERVO', VisualServo(), 
                 transitions={'navigate':'GOAL'})
    smach.StateMachine.add('GOAL', NavigateGoal(), 
                 transitions={'start':'START'})
    smach.StateMachine.add('START', NavigateStart(), 
                 transitions={'turn':'TURN', "finish": "finish"})
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