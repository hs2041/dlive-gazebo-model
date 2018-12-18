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

v = np.array([1, 0, 0])
setval = -90
t1 = time.time()

def imu_finder(msg):
	global t1,setval

	t2 = time.time()

	if t2-t1 > 30:
		setval = 0
	elif t2-t1 > 20:
		setval = 90
	elif t2-t1 > 10:
		setval = 0
	#global v
	
	noise = np.zeros(4,dtype = Float64)

	#Adding error to the orientation
	noise=np.random.normal(0,0.1,4) 

	q1 = Quaternion(msg.orientation.w +noise[0],msg.orientation.x +noise[1],msg.orientation.y +noise[2],msg.orientation.z +noise[3])

	v1 = q1.rotate(v)

	theta = math.atan2(v1[1],v1[0])

	setter.publish(setval)
	stater.publish(int(theta * 180/np.pi) + 3)

	#print (theta * 180/np.pi)

if __name__ == '__main__':
	
	rospy.init_node('imu_tester')

	rospy.Subscriber('imu', Imu, imu_finder)

	setter = rospy.Publisher('setpoint',Float64,queue_size = 1)
	stater = rospy.Publisher('state',Float64,queue_size = 1)	
	r = rospy.Rate(100)

#	while not rospy.is_shutdown():
	rospy.spin()		