#!/usr/bin/env python
# launch joy node
# rosrun joy joy_node
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
import math
import random

def sum(s):
    total = 0
    for i in range(0, len(s)):
        if not math.isnan(s[i]):
            total += s[i]
    return total

def scan_callback(msg):
    global g_range_ahead
    global ranges
    g_range_ahead = min(msg.ranges)
    ranges = msg.ranges
    # print(type(msg.ranges))

def joy_callback(msg):
    global start
    if msg.buttons[0] == 1:
        rospy.loginfo("start pressed!")
        start = not start
g_range_ahead = 1  # anything to start
rospy.Subscriber("scan", LaserScan, scan_callback)
rospy.Subscriber("joy", Joy, joy_callback)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
rospy.init_node("wander")
state_change_time = rospy.Time.now() + rospy.Duration(random.randint(4,5))
driving_forward = True
turn_left = False
turn_right = False
rate = rospy.Rate(10)
global start, ranges
start = False
while not rospy.is_shutdown():
    # print(g_range_ahead)
    if start:
        twist = Twist()
        if driving_forward:
            print("go forward")
            # BEGIN FORWARD
            twist.linear.x = 0.5
            twist.angular.z = 0
            
            if (rospy.Time.now() > state_change_time):
                state_change_time_temp = rospy.Time.now() + rospy.Duration(1)
                direction = random.randint(1,2) 
                while (rospy.Time.now() < state_change_time_temp):
                    print "random turn"
                    twist = Twist()
                    if (direction == 2):
                        twist.angular.z =  -1.5
                    else:
                        twist.angular.z =  1.5
                    cmd_vel_pub.publish(twist)
                state_change_time = rospy.Time.now() + rospy.Duration(random.randint(4,5))


            if (min(ranges[100:540]) < 1):
                driving_forward = False
                twist.linear.x = 0
                print(sum(ranges[0:320]), sum(ranges[320:639]))
                if sum(ranges[0:320])<sum(ranges[320:639]):
                    turn_left = True
                else:
                    turn_right = True
            # END FORWARD
        # else:  # we're not driving_forward
        #     # BEGIN TURNING
        #     twist.linear.x = 0.1
        #     twist.angular.z = 1
        #     if g_range_ahead > 0.8 or rospy.Time.now() > state_change_time :
        #         driving_forward = True  # we're done spinning, time to go forwards!
        #         state_change_time = rospy.Time.now() + rospy.Duration(30)
        #         twist.angular.z = 0
        #     # END TURNING
        if turn_left:
            print("turn left")
            twist.linear.x = 0.1
            twist.angular.z = 1.5
            if min(ranges[100:540]) > 0.8:
                driving_forward = True  # we're done spinning, time to go forwards!
                state_change_time = rospy.Time.now() + rospy.Duration(4)
                twist.angular.z = 0
                turn_left = False
        if turn_right:
            print("turn right")
            twist.linear.x = 0.1
            twist.angular.z = -1.5
            if min(ranges[100:540]) > 0.8:
                driving_forward = True  # we're done spinning, time to go forwards!
                state_change_time = rospy.Time.now() + rospy.Duration(4)
                twist.angular.z = 0
                turn_right = False
        cmd_vel_pub.publish(twist)

    rate.sleep()
# END ALL
