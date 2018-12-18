#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
		closest_range = min(msg.ranges)
		range_ahead = msg.ranges[len(msg.ranges)/2]
		print "range ahead: %0.1f" % range_ahead

rospy.init_node('range_ahead1')
scanner = rospy.Subscriber('scan',LaserScan, scan_callback)
rospy.spin()
