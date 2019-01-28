#!/usr/bin/env python

import sys, select, tty, termios
import rospy
import math
from geometry_msgs.msg import Twist

key_map = { 'w': [0, 1], 's': [0, -1],
            'a': [1, 0], 'd': [-1, 0],
            ' ': [0, 0]}

g_vel_ramps = [0.1, 0.1]

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step: # done within this timestep
        return v_target
    else: 
        return v_prev + sign * step

def ramped_twist(prev, target, t_prev, t_now, ramps):
    t = Twist()
    t.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev, t_now, ramps[0])
    t.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, ramps[1])
    return t

def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist, g_vel_ramps
    
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist, g_last_twist_send_time, t_now, g_vel_ramps)
    g_last_twist_send_time = t_now
    twist_pub.publish(g_last_twist)
    
def handle_key(key):
    global g_target_twist, g_last_twist
    if not key_map.has_key(key):
        return
    vels = key_map[key]
    g_target_twist.angular.z = vels[0]
    g_target_twist.linear.x = 0

if __name__ == '__main__':
    rospy.init_node("teleop")
    rate = rospy.Rate(100)
    
    twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    print "Starting teleop node"
    g_last_twist = Twist()
    g_target_twist = Twist()
    g_last_twist_send_time = rospy.Time.now()

    
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            handle_key(sys.stdin.read(1))
        send_twist()
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
