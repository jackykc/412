#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np

bridge = cv_bridge.CvBridge()

# mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
mtx = np.array([[522.0342478023756, 0, 314.8622376912774], [0, 522.0984910339546, 254.7467445228038], [0, 0, 1]], np.float32)
dist = np.array([0.01429000810948565, -0.08328252729130574, 0.0003490591084126651, 0.0006503454262249897, 0], np.float32)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((7*10,3), np.float32)
objp[:,:2] = np.mgrid[0:10,0:7].T.reshape(-1,2)

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def img_callback(msg):

    img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (10,7),None)
    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        # project 3D points to img plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        img = draw(img,corners2,imgpts)
 
    img_pub.publish(bridge.cv2_to_imgmsg(img, encoding='bgr8'))



rospy.init_node('part1')
bridge = cv_bridge.CvBridge()
img_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, img_callback)
img_pub = rospy.Publisher('transformed_img', Image, queue_size=1)
rospy.spin()