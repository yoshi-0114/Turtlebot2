#! /usr/bin/env python3
  
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import subprocess
import os
  
class turtleBot2:
    WIDTH = 12
    HEIGHT = 30
    
    def __init__(self):
        rospy.init_node('move_turtlebot2', anonymous=True)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1000)
        rospy.Subscriber('camera/depth/image_raw',Image, self.depthCallback)
        self.bridge = CvBridge()
        self.a = 1
        self.ave = 0.0
  
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)
  
    def depthCallback(self, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, '32FC1')
        except CvBridgeError as e:
            rospy.logerr(e)

        h, w = depth_image.shape
 
        x1 = int(w / 2) - self.WIDTH
        x2 = int(w / 2) + self.WIDTH
        y1 = int(3*h / 4) - self.HEIGHT
        y2 = int(3*h / 4) + self.HEIGHT
        sum = 0.0
 
        for i in range(y1, y2):
            for j in range(x1, x2):
                if depth_image.item(i,j) == depth_image.item(i,j):
                    sum += depth_image.item(i,j)
 
        ave = (sum / ((self.WIDTH * 2) * (self.HEIGHT * 2))) * 0.001
        self.ave = ave
        print("%f [m]" % ave)
 
        cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)
        cv2.namedWindow("depth_image")
        cv2.imshow("depth_image", depth_image)
        cv2.waitKey(100)
  
    def run(self):
        r = rospy.Rate(10)

        rospy.sleep(1)
        while True:
            if self.ave < 0.85:
                print(self.a)
                if self.a == 3:
                    rospy.sleep(1.0)
                    self.setMoveVector(-0.2,30)
                    rospy.sleep(1.0)
                    os.chdir('kobuki_auto_docking/launch')
                    subprocess.call(["roslaunch","activate.launch"])
                    sys.exit(0)
                if self.a % 2 == 0:
                    self.setRotateVector(1.564, 10)
                else:
                    self.setRotateVector(-1.564, 10)
                self.a += 1
            else:
                self.setMoveVector(0.2, 1)
            
    def setMoveVector(self, linear_x, cnt):
        twist = Twist()
        r = rospy.Rate(10)
  
        twist.linear.x = linear_x
        twist.angular.z = 0.0
        
        for i in range(0,cnt): 	
       	 self.twist_pub.publish(twist)
       	 r.sleep()
    
    def setRotateVector(self, angular_z, cnt):
        twist = Twist()
        r = rospy.Rate(10)
        
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        
        rospy.sleep(1.0)
        for i in range(0,cnt): 	
       	 self.twist_pub.publish(twist)
       	 r.sleep()
       	 
        rospy.sleep(3.0)
        self.resetVector()
        
    def resetVector(self):
        twist = Twist()
        
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.twist_pub.publish(twist)
        rospy.sleep(0.1)
        
if __name__ == '__main__':
  
    try:
        ts = turtleBot2()
        ts.run()
        rospy.spin()
    except rospy.ROSInterruptException: pass
