#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2, cv_bridge, numpy
import smach
import smach_ros

global stop, donot_check_time, image_pub, err, cmd_vel_pub, bridge, stop_count, line_lost
line_lost = False
stop_count = 0
rospy.init_node('follower')
err = 0
def image_callback(msg):
    global stop, donot_check_time, image_pub, err, line_lost
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = numpy.array([0, 0,  242])
    upper_white = numpy.array([170, 50, 256])
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([200, 256, 256])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    masked = cv2.bitwise_and(image, image, mask=mask_red)
    # image_pub.publish(bridge.cv2_to_imgmsg(masked, encoding='bgr8'))

    # check red line
    h, w, d = image.shape
    search_top = h-70
    search_bot = h-50
    mask_red[0:search_top, 0:w] = 0
    mask_red[search_bot:h, 0:w] = 0
    M = cv2.moments(mask_red)
    if M['m00'] > 0 and rospy.Time.now() > donot_check_time:
        stop = True
        donot_check_time = rospy.Time.now()+rospy.Duration(5)
    if stop:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,255,0), -1)
        image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        return

    # masked = cv2.bitwise_and(image, image, mask=mask)
    # image_pub.publish(bridge.cv2_to_imgmsg(masked, encoding='bgr8'))

    # track white line
    h, w, d = image.shape
    search_top = h-70
    search_bot = h-50
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
        line_lost = False
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        err = cx - w/2
    else:
        line_lost = True
    image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))


bridge = cv_bridge.CvBridge()
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
image_pub = rospy.Publisher('transformed_img', Image, queue_size=1)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
# cmd_vel_pub = rospy.Publisher('/teleop_velocity_smoother/raw_cmd_vel', Twist, queue_size=1)
donot_check_time = rospy.Time.now()
stop = False

class Go(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])
        self.twist = Twist()
    def execute(self, data):
        global stop, err, cmd_vel_pub, stop_count
        while not rospy.is_shutdown():
            if stop:
                stop = False
                return 'stop'
            else:
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 200
                cmd_vel_pub.publish(self.twist)

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go', 'task1', 'task2', 'task3'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub, stop_count
        stop_count += 1
        # go a bit further
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        # speed = 0.2
        # while speed > 0:
        #     self.twist.linear.x = 0.2
        #     self.twist.angular.z = 0
        #     cmd_vel_pub.publish(self.twist)
        #     speed -= 0.00001
        # determin which it is
        if stop_count == 1:
            return 'task1'
        elif stop_count == 3:
            return 'task2'
        elif stop_count == 5:
            return 'task3'
        # regular stop
        wait_time = rospy.Time.now() + rospy.Duration(2)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        return 'go'

class Task1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = -1.5
            cmd_vel_pub.publish(self.twist)
        return 'go'

class Task2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub, err,  line_lost
        print 'in task 2'
        wait_time = rospy.Time.now() + rospy.Duration(1.5)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1.4)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        # track the line
        print 'tracking line'
        while (not rospy.is_shutdown()) and (not line_lost):
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 200
            cmd_vel_pub.publish(self.twist)
        # reaches the end, stop for 2 second
        print 'reaches the end'
        wait_time = rospy.Time.now() + rospy.Duration(2)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        # turn back
        wait_time = rospy.Time.now() + rospy.Duration(2.8)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        # track the line
        stop = False
        while (not rospy.is_shutdown()) and not stop:
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 200
            cmd_vel_pub.publish(self.twist)
        # stops at red line, return to go state
        wait_time = rospy.Time.now() + rospy.Duration(3)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        stop = False
        return 'go'

class Task3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        global stop, cmd_vel_pub
        for i in range(0, 2):
            # track the line
            stop = False
            while (not rospy.is_shutdown()) and not stop:
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 200
                cmd_vel_pub.publish(self.twist)
            
            # check each one
            wait_time = rospy.Time.now() + rospy.Duration(1.4)
            while rospy.Time.now()<wait_time:
                self.twist.linear.x = 0
                self.twist.angular.z = 1.5
                cmd_vel_pub.publish(self.twist)
            wait_time = rospy.Time.now() + rospy.Duration(1)
            while rospy.Time.now()<wait_time:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                cmd_vel_pub.publish(self.twist)
            wait_time = rospy.Time.now() + rospy.Duration(1.4)
            while rospy.Time.now()<wait_time:
                self.twist.linear.x = 0
                self.twist.angular.z = -1.5
                cmd_vel_pub.publish(self.twist)
        # check last one
        wait_time = rospy.Time.now() + rospy.Duration(2)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1.4)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 1.5
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            cmd_vel_pub.publish(self.twist)
        wait_time = rospy.Time.now() + rospy.Duration(1.4)
        while rospy.Time.now()<wait_time:
            self.twist.linear.x = 0
            self.twist.angular.z = -1.5
            cmd_vel_pub.publish(self.twist)
        stop = False
        return 'go'

# follower = Follower()
sm = smach.StateMachine(outcomes=['finish'])
with sm:
    # Add states to the container
    smach.StateMachine.add('GO', Go(), 
                 transitions={'stop':'STOP'})
    smach.StateMachine.add('STOP', Stop(), 
                 transitions={'go':'GO', 'task1': 'TASK1', 'task2': 'TASK2', 'task3': 'TASK3'})
    smach.StateMachine.add('TASK1', Task1(), 
                 transitions={'go':'GO'})
    smach.StateMachine.add('TASK2', Task2(), 
                 transitions={'go':'GO'})
    smach.StateMachine.add('TASK3', Task3(), 
                 transitions={'go':'GO'})
# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

outcome = sm.execute()
rospy.spin()
sis.stop()