#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float64
from ackermann_msgs.msg import AckermannDrive
import numpy as np 
import math
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from pyquaternion import Quaternion

timer1 = time.time()

t1 = 0
t2 = 0
sign = 1
count = 0
time_total = 0

def state_finder(msg):
	global t1
	t1 = msg.data
def set_finder(msg1):
	global t2,sign,timer1, count, time_total
	t2 = msg1.data
	
	timer2 = time.time()

	if np.sign(t1-t2) == -sign:
		sign = -sign
		time_total += timer2 - timer1
		count +=1
		timer1 = timer2

		print(time_total/count, " ", count, " ", timer2 - timer1)
	#print (theta * 180/np.pi)

if __name__ == '__main__':


	rospy.init_node('period_finder')

	rospy.Subscriber('setpoint', Float64, set_finder)
	rospy.Subscriber('state', Float64, state_finder)
	
	
#	while not rospy.is_shutdown():
	rospy.spin()		