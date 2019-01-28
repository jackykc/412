#!/usr/bin/env python
import random

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, Led
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import decompose_matrix, euler_from_quaternion
from ros_numpy import numpify

global current_angle
global bumper_hit
global cur_dist
cur_dist = None
def calc_delta_vector(start_heading, distance):
    dx = distance * np.cos(start_heading)
    dy = distance * np.sin(start_heading)
    return np.array([dx,dy])      

def check_forward_distance(forward_vec, start_pos, current_pos):
    current = current_pos - start_pos
    #vector projection
    delta = np.dot(current, forward_vec) / np.dot(forward_vec, forward_vec) * forward_vec
    dist = np.sqrt(delta.dot(delta))
    return dist

def odom_callback(msg):
  global current_angle, cur_pos, cur_heading, cur_dist

  pose_msg = msg.pose.pose
  pose = numpify(pose_msg) 
  
  __, __, angles, translate, __ = decompose_matrix(pose)
  current_angle = angles[2] + 3.14159
  cur_pos = translate[0:2]
  cur_heading = angles[2]
  cur_dist = check_forward_distance(delta,start_pos,cur_pos)
  # rospy.loginfo(cur_dist)

def bumper_callback(msg):
  global bumper_hit
  bumper_hit = (msg.bumper == 1) and msg.state
  # rospy.loginfo(str(msg.bumper) + " " + str(msg.state))

bumper_hit = False
bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
odom_sub = rospy.Subscriber("odom",Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

class DriveFoward(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['driveBackward', 'end'])

  def execute(self, userdata):
    global bumper_hit, cur_dist
    rospy.loginfo('Executing state DRIVE_FOWARD')
    
    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    
    while (not bumper_hit):
      if cur_dist is not None and cur_dist > 3.5:
        return 'end'
      twist = Twist()
      twist.linear.x = 0.2
      cmd_vel_pub.publish(twist)
    
    return 'driveBackward'

class Start(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['driveForward'])
  def execute(self, userdata):
    return 'driveFoward'


class DriveBackward(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['turnLeft'])

  def execute(self, userdata):
    rospy.loginfo('Executing state DRIVE_BACKWARD')
    state_change_time = rospy.Time.now() + rospy.Duration(3)
    
    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    
    while (rospy.Time.now() < state_change_time):
      twist = Twist()
      twist.linear.x = -0.2
      cmd_vel_pub.publish(twist)
    
    return 'turnLeft'

class DriveBackwardMore(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['turnRight'])

  def execute(self, userdata):
    rospy.loginfo('Executing state DRIVE_BACKWARD')
    state_change_time = rospy.Time.now() + rospy.Duration(5)
    
    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    
    while (rospy.Time.now() < state_change_time):
      twist = Twist()
      twist.linear.x = -0.2
      cmd_vel_pub.publish(twist)
    
    return 'turnRight'

class End(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['finished'])
  def execute(self, userdata):
    turnoff_time = rospy.Time.now()+rospy.Duration(5)
    while rospy.Time.now() < turnoff_time:
      led = rospy.Publisher('/mobile_base/commands/led1', Led)
      led.publish(Led(2))
    return 'finished'

class DriveForwardLess(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['turnRight', 'driveBackMore'])

  def execute(self, userdata):
    global bumper_hit
    rospy.loginfo('Executing state DRIVE_FORWARD_LESS')
    state_change_time = rospy.Time.now() + rospy.Duration(4)
    
    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    
    while (rospy.Time.now() < state_change_time):
      if bumper_hit:
        return 'driveBackMore'
      twist = Twist()
      twist.linear.x = 0.2
      cmd_vel_pub.publish(twist)
    
    return 'turnRight'

class TurnRight(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['driveFoward'])

  def execute(self, userdata):
    global current_angle
    # angle_goal = (current_angle - (90 * 3.14159 / 180))%(2*3.14159)
    angle_goal = 3.14159
    rospy.loginfo(current_angle)
    rospy.loginfo(angle_goal)
    rospy.loginfo('Executing state TURN_RIGHT')
    state_change_time = rospy.Time.now() + rospy.Duration(3)
    angular_speed = 0.5 * 3.14159 / 180

    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    

    while ((current_angle < angle_goal - 0.03) or (current_angle > angle_goal + 0.03)):
      twist = Twist()
      twist.angular.z = -0.5
      # rospy.loginfo(current_angle)
      # rospy.loginfo(angle_goal)
      cmd_vel_pub.publish(twist)
    
    return 'driveFoward'

class TurnLeft(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['driveFowardLess'])

  def execute(self, userdata):
    global current_angle
    angle_goal = (current_angle + (90 * 3.14159 / 180))%(2*3.14159)
    rospy.loginfo(current_angle)
    rospy.loginfo(angle_goal)
    rospy.loginfo('Executing state TURN_LEFT')
    state_change_time = rospy.Time.now() + rospy.Duration(3)
    angular_speed = 0.5 * 3.14159 / 180

    twist = Twist()
    twist.angular.x = 0
    twist.angular.z = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.linear.x = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    

    while ((current_angle < angle_goal - 0.01) or (current_angle > angle_goal + 0.01)):
      twist = Twist()
      twist.angular.z = 0.5
      # rospy.loginfo(current_angle)
      # rospy.loginfo(angle_goal)
      cmd_vel_pub.publish(twist)
    
    return 'driveFowardLess'


# main
def main():
  rospy.init_node('smach_example_state_machine')
  global start_pos, delta, start_heading
  start_pos = None
  start_heading = None
  delta = None
  while not rospy.is_shutdown():
    if start_pos is None:
      start_pos = cur_pos
      start_heading = cur_heading
      delta = calc_delta_vector(start_heading,1)
      break
  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['finished'])

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add('DRIVE_FORWARD', DriveFoward(), 
                 transitions={'driveBackward':'DRIVE_BACKWARD', 'end': 'END'})
    smach.StateMachine.add('DRIVE_BACKWARD', DriveBackward(), 
                 transitions={'turnLeft':'TURN_LEFT'})
    smach.StateMachine.add('DRIVE_BACKWARD_MORE', DriveBackwardMore(), 
                 transitions={'turnRight':'TURN_RIGHT'})
    smach.StateMachine.add('DRIVE_FORWARD_LESS', DriveForwardLess(), 
                 transitions={'turnRight':'TURN_RIGHT', 'driveBackMore':'DRIVE_BACKWARD_MORE'})
    smach.StateMachine.add('TURN_RIGHT', TurnRight(), 
                 transitions={'driveFoward':'DRIVE_FORWARD'})
    smach.StateMachine.add('TURN_LEFT', TurnLeft(), 
                 transitions={'driveFowardLess':'DRIVE_FORWARD_LESS'})
    smach.StateMachine.add('END', End(), 
                 transitions={})
  # Create and start the introspection server
  # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  # sis.start()

  outcome = sm.execute()
  # Wait for ctrl-c to stop the application
  rospy.spin()
  # sis.stop()


if __name__ == '__main__':
  main()