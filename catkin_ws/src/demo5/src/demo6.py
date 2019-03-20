#!/usr/bin/env python
# Retrieved from https://github.com/osrf/rosbook/blob/master/code/navigation/src/patrol.py added additional goals
import rospy
import actionlib

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import numpy as np
from ros_numpy import numpify, geometry

from tf.transformations import decompose_matrix, euler_from_quaternion

from sensor_msgs.msg import Image
import cv2
import cv_bridge


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan, Joy

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
mtx = np.array([[522.0342478023756, 0, 314.8622376912774], [0, 522.0984910339546, 254.7467445228038], [0, 0, 1]], np.float32)
dist = np.array([0.01429000810948565, -0.08328252729130574, 0.0003490591084126651, 0.0006503454262249897, 0], np.float32)

bridge = cv_bridge.CvBridge()


def draw(img, corner, imgpts):
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

global np_current_marker_pose
np_current_marker_pose = None
def marker_cb(msg):
    global np_current_marker_pose
    if len(msg.markers):
        # print msg.markers

        for marker in msg.markers:
            current_marker_pose = marker.pose.pose
            np_current_marker_pose = numpify(current_marker_pose)


def img_callback(msg):
    global np_current_marker_pose
    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if np_current_marker_pose is not None:
        scale, shear, rpy_angles, translation_vector, perspective = tf.transformations.decompose_matrix(np_current_marker_pose)
        print rpy_angles, translation_vector
        
        imgpts, jac = cv2.projectPoints(axis, np.asarray(rpy_angles), np.asarray(translation_vector), mtx, dist)


        img = draw(img,(20, 20),imgpts)

        img_pub.publish(bridge.cv2_to_imgmsg(img, encoding='bgr8'))
'''
scale, shear, rpy_angles, translation_vector, perspective = tf.transformations.decompose_matrix(np_current_marker_pose)
            print rpy_angles, translation_vector
            imgpts, jac = cv2.projectPoints(axis, rpy_angles, translation_vector, mtx, dist)


            img = draw(img,(200, 200),imgpts)
    '''            

rospy.init_node('patrol')

marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_cb)
img_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, img_callback)
img_pub = rospy.Publisher('transformed_img', Image, queue_size=1)

rospy.spin()