#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge, numpy
from detect.detect_lib import detect_1, detect_2, detect_3
from dynamic_reconfigure.server import Server
from demo5.cfg import HSVConfig
global bridge, action
global red, green
red = []
red.append(numpy.array([130, 132,  110]))
red.append(numpy.array([180, 255, 255]))

action = 0
# 0 1 2 3
    

def clamp_count(count):
    if count < 1:
        return 1
    elif count > 3:
        return 3
    else:
        return count

def get_shape_id(vertices):
    if vertices == 3:
        id = 0
    elif vertices == 4:
        id = 1
    else:
        id = 2

    return id

def get_shape(shape_id):
    if shape_id == 0:
        shape = "triangle"
    elif shape_id == 1:
        shape = "square"
    else:
        shape = "circle"

    return shape

def get_vertices(contours):
    approx = []
    areas = [cv2.contourArea(c) for c in contours]
    if len(areas):
        max_index = numpy.argmax(areas)
        largest_contour = contours[max_index]

        peri = cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)
    return len(approx)

def image_callback(msg):
    global red
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    if (action == 0):
        image, count = detect_1(image, red)
        print str(count)
    elif (action == 1):
        image, count, shape_id = detect_2(image, red)
        print str(count) + " " + str(get_shape(shape_id))
    elif (action == 2):
        image, count, shape_id = detect_3(image, red)
        print str(count) + " " + str(get_shape(shape_id))

    image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))

def reconfigure_callback(config, leven):
    global red, action
    action = config.detect_type
    red[0][0] = config.low_h
    red[0][1] = config.low_s
    red[0][2] = config.low_v
    red[1][0] = config.high_h
    red[1][1] = config.high_s
    red[1][2] = config.high_v
    
    return config

print "hi"
rospy.init_node('detector')
print "why"
bridge = cv_bridge.CvBridge()
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
image_pub = rospy.Publisher('/transformed_img', Image, queue_size=1)
srv = Server(HSVConfig, reconfigure_callback)


rospy.spin()