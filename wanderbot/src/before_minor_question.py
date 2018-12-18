#!/usr/bin/env python
import rospy
import numpy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg, twist_pub):

	range_ahead = msg.ranges[len(msg.ranges)/2]

	print(math.isnan(range_ahead))

def scan_callback(msg, twist_pub):
		
	range_ahead = msg.ranges[len(msg.ranges)/2]
	a = Twist()
	checker = math.isnan(range_ahead)

	if checker:
		a.linear.x = 0.3

	else
		
	range_aheadT = msg.ranges[(len(msg.ranges)/2)+10]
	range_aheadF = msg.ranges[(len(msg.ranges)/2)-10]

	print("Length of scan",len(msg.ranges))
	b = msg.ranges.index(max(msg.ranges))
	print("indexed value = ",b)
	
	if (range_ahead > range_aheadF) and (range_ahead > range_aheadT):
		pass
		#a.linear.x = 
	elif range_ahead < range_aheadF:
		a.angular.z = -0.2
		print('a')

	elif range_ahead < range_aheadT:
		a.angular.z = 0.2
		print('b')

	print( range_ahead)
	if range_ahead > 0.5 or range_ahead == 'nan':
		a.linear.x = 0.2
		print('c')

	else:
		a.linear.x = 0
		print('d')

	#print(range_ahead)
	twist_pub.publish(a)
		

if __name__ == '__main__':
	rospy.init_node('basic_auto')
	twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
	scanner = rospy.Subscriber('scan',LaserScan, scan_callback, twist_pub)

	rospy.spin()