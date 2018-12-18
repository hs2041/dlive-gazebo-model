#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size =1)
rospy.init_node('red_and_green_light')

red_light_twist = Twist()
green_light_twist = Twist()

green_light_twist.linear.x = 0.5

driving_forward = False
light_change_time = rospy.Time.now()

rate = rospy.Rate(5)

while not rospy.is_shutdown():
	if driving_forward:
		cmd_vel_pub.publish(green_light_twist)
		rospy.loginfo(green_light_twist.linear.x)
	else:
		cmd_vel_pub.publish(red_light_twist)
		rospy.loginfo(red_light_twist.linear.x)

	if( light_change_time < rospy.Time.now()):
		driving_forward = not driving_forward
		light_change_time = rospy.Time.now() + rospy.Duration(2)
	rate.sleep()

