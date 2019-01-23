#!/usr/bin/env python
import random

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry

from tf.transformations import decompose_matrix, euler_from_quaternion
from ros_numpy import numpify

global current_angle
global bumper_hit

def odom_callback(msg):
  global current_angle

  pose_msg = msg.pose.pose
  pose = numpify(pose_msg) 
  
  __, __, angles, translate, __ = decompose_matrix(pose)
  current_angle = angles[2] + 3.14159

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
    smach.State.__init__(self, outcomes=['driveBackward'])

  def execute(self, userdata):
    global bumper_hit
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

class DriveForwardLess(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['turnRight'])

  def execute(self, userdata):
    rospy.loginfo('Executing state DRIVE_FORWARD_LESS')
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
    

    while ((current_angle < angle_goal - 0.03) or (current_angle > angle_goal + 0.03)):
      twist = Twist()
      twist.angular.z = 0.5
      # rospy.loginfo(current_angle)
      # rospy.loginfo(angle_goal)
      cmd_vel_pub.publish(twist)
    
    return 'driveFowardLess'


# main
def main():
  rospy.init_node('smach_example_state_machine')
  
  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['end'])

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add('DRIVE_FORWARD', DriveFoward(), 
                 transitions={'driveBackward':'DRIVE_BACKWARD'})
    smach.StateMachine.add('DRIVE_BACKWARD', DriveBackward(), 
                 transitions={'turnLeft':'TURN_LEFT'})
    smach.StateMachine.add('DRIVE_FORWARD_LESS', DriveForwardLess(), 
                 transitions={'turnRight':'TURN_RIGHT'})
    smach.StateMachine.add('TURN_RIGHT', TurnRight(), 
                 transitions={'driveFoward':'DRIVE_FORWARD'})
    smach.StateMachine.add('TURN_LEFT', TurnLeft(), 
                 transitions={'driveFowardLess':'DRIVE_FORWARD_LESS'})
  
  # Create and start the introspection server
  # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  # sis.start()

  outcome = sm.execute()
  # Wait for ctrl-c to stop the application
  rospy.spin()
  # sis.stop()


if __name__ == '__main__':
  main()