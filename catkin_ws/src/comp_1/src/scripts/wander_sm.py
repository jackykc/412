#!/usr/bin/env python
import random

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
  global g_range_ahead
  g_range_ahead = min(msg.ranges)

g_range_ahead = 1 # anything to start
g_range_ahead_thresh = None
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

state_change_time = None

# define state Foo
class Drive(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['turning'])

  def execute(self, userdata):
    global state_change_time, cmd_vel_pub, g_range_ahead_thresh
    rospy.loginfo('Executing state DRIVE')
    rospy.loginfo(g_range_ahead)
    rospy.loginfo(rospy.Time.now())
    rospy.loginfo(state_change_time)


    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    
    while ((g_range_ahead > g_range_ahead_thresh) and (rospy.Time.now() < state_change_time)):
      twist = Twist()
      twist.linear.x = 1
      cmd_vel_pub.publish(twist)
    state_change_time = rospy.Time.now() + rospy.Duration(random.randint(1,5))
    return 'turning'


# define state Bar
class Turn(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['driving'])

  def execute(self, userdata):
    global state_change_time, cmd_vel_pub
    rospy.loginfo('Executing state TURN')
    rospy.loginfo(rospy.Time.now())
    rospy.loginfo(state_change_time)
  
    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    
    while rospy.Time.now() < state_change_time:
      twist = Twist()
      twist.angular.z = 0.5
      cmd_vel_pub.publish(twist)
    state_change_time = rospy.Time.now() + rospy.Duration(30)
    return 'driving'

# main
def main():
  rospy.init_node('smach_example_state_machine')
  global state_change_time, g_range_ahead_thresh
  state_change_time = rospy.Time.now()
  g_range_ahead_thresh = rospy.get_param('~g_range_ahead')

  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=[])

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add('DRIVE', Drive(), 
                 transitions={'turning':'TURN'})
    smach.StateMachine.add('TURN', Turn(), 
                 transitions={'driving':'DRIVE'})

  
  # Create and start the introspection server
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  outcome = sm.execute()

  # Wait for ctrl-c to stop the application
  rospy.spin()
  sis.stop()


if __name__ == '__main__':
  main()