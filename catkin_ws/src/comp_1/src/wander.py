#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)


g_range_ahead = 1  # anything to start
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
rospy.init_node("wander")
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    twist = Twist()
    if driving_forward:
        # BEGIN FORWARD
        twist.linear.x = 0.3
        twist.angular.z = 0
        if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(5)
            twist.linear.x = 0
        # END FORWARD
    else:  # we're not driving_forward
        # BEGIN TURNING
        twist.angular.z = 1
        twist.linear.x = 0
        if rospy.Time.now() > state_change_time:
            driving_forward = True  # we're done spinning, time to go forwards!
            state_change_time = rospy.Time.now() + rospy.Duration(30)
            twist.angular.z = 0
        # END TURNING
    cmd_vel_pub.publish(twist)

    rate.sleep()
# END ALL
