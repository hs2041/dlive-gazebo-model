#!/usr/bin/env python

#more forward until obstacle more than 0.8 meters or 30 seconds passed then move in a different direction
import rospy
import time
import cv2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

start_time = time.time()
a = Twist()
def scan_callback(msg):
		global start_time
		range_ahead = msg.ranges[len(msg.ranges)/2]
		print "range ahead: %0.1f" % range_ahead
		a.angular.z = 0
		a.linear.x = 0.5
		
		if range_ahead == 'nan':
		 	a.linear.x = 0.2
		if (range_ahead < 0.8) or (start_time+5 < time.time()):
			a.linear.x = 0
			a.angular.z = 1
			print('sdjlkjds')
			start_time = time.time()
		
		pub.publish(a)
rospy.init_node('range_ahead')
scanner = rospy.Subscriber('scan',LaserScan, scan_callback)
pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist,queue_size = 1)
rospy.spin()
