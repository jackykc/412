#!/usr/bin/env python
import rospy
from tf.transformations import decompose_matrix, euler_from_quaternion
from ros_numpy import numpify
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
# import numpy as np

# print np

START = True
cur_pos = None
cur_heading = None
'''
def joy_callback(msg): 
    global START
    if msg.buttons[0] == 1:
        START = True

def odom_callback(msg):
    global cur_pos, cur_heading
    pose_msg = msg.pose.pose
    pose = numpify(pose_msg) 
    __, __, angles, translate, __ = decompose_matrix(pose)

    cur_pos = translate[0:2]
    cur_heading = angles[2]

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
'''
if __name__ == "__main__":
    # start_pos = None
    # start_heading = None
    # delta = None
    rospy.init_node("odom_node")
    # rospy.Subscriber("odom",Odometry, odom_callback)
    # rospy.Subscriber("joy",Joy, joy_callback)
    # twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        led = rospy.Publisher('/mobile_base/commands/led1', Led)
        led.publish(Led(0))
        # break
        '''
        if START:
            if start_pos is None:
                start_pos = cur_pos
                start_heading = cur_heading
                delta = calc_delta_vector(start_heading,1)

            cur_dist = check_forward_distance(delta,start_pos,cur_pos)             
            print(cur_dist)
            if cur_dist > 1:
                break
            
            # p controller to maintain heading
            # err = cur_heading - start_heading

            msg = Twist()
            msg.linear.x = 0.3
            msg.angular.z = 0
            twist_pub.publish(msg) 
        '''

        
        rate.sleep()
    rospy.spin()