#!/usr/bin/env python
import rospy
from basics.msg import msg_template
from random import random

rospy.init_node('message_publisher')
pub = rospy.Publisher('message_template', msg_template, queue_size = 2)

rate = rospy.Rate(2)
msg = msg_template()
i = 1
while not rospy.is_shutdown():
	msg.data1 = i
	i += 1
	pub.publish(msg)

rate.sleep()