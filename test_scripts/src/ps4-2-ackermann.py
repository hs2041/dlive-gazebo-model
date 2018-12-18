#!/usr/bin/env python
import rospy
import time
import threading
import math
import numpy as np
from e2o.msg import e2o_ctrl, e2o_status
from ackermann_msgs.msg import AckermannDrive



car_ctrl = e2o_ctrl()
car_ctrl.Accel = 0
car_ctrl.Brake = 0
car_ctrl.Steer = 0
car_ctrl.RNDB = 'N'

ackermann_ctrl = AckermannDrive()
ackermann_ctrl.steering_angle_velocity = 1
ackermann_ctrl.speed = 0 
ackermann_ctrl.steering_angle = 0 
ackermann_ctrl.acceleration = 0
ackermann_ctrl.jerk = 0 

last_speed = 0.0
a1	= 0.2			# if input acceleration is "'a' then top speed that can be reached is a*a1
a2	= 0.02/50		# increment speed by a2*a
a3	= 0.005/50		# decrement speed by a3*a
b1	= 1.0/1500		# decrement speed by b1*brake %
c1	= 1.0/1000		# when gear is neutral, decrement speed by c1


print("a1 ", a1)
print("a2 ", a2)
print("a3 ", a3)
print("b1 ", b1)
print("c1 ", c1)

#=====================================================================

def e2o_ctrl_callback(car_status):
	#print("Inside callback")
	global car_control

	global ackermann_ctrl,last_speed
	
	ackermann_ctrl.steering_angle = car_status.Steer * np.pi/180

	#Set the steering angle
	if abs(ackermann_ctrl.steering_angle) > 40*np.pi/180:
		
		ackermann_ctrl.steering_angle = -np.sign(ackermann_ctrl.steering_angle) *40*np.pi/180

	#at any point of time one of the brakes and acceleration is equal to zero
	
	#Set the brakes
	
	if car_status.RNDB == 'N' and car_status.Brake == 0:
		speed = last_speed - c1

	if car_status.Accel > 0 and car_status.RNDB != 'N':
		
			#if abs(last_speed - a1*car_control.Accel) < a2*car_control.Accel:
			#	speed = a1*car_control.Accel

			if last_speed > a1*car_status.Accel:
				speed = last_speed - a3*car_status.Accel
			elif last_speed < a1*car_status.Accel:
				speed = last_speed + a2*car_status.Accel
			else:
				speed = last_speed

	if car_status.Brake > 0:
		speed = last_speed - b1*car_status.Brake

		if speed<0:
			speed = 0
	
	elif (car_status.Accel == 0 and car_status.Brake == 0) :
		speed = last_speed - c1
		if speed < 0:
			speed = 0


	
	last_speed = speed
	'''
	if car_status.RNDB == 'R':
		speed = speed * (-1)
	'''
	ackermann_ctrl.speed = speed

	#car_drive.publish(ackermann_ctrl)
#=====================================================================
#ROS NODE : SUBSCRIBERS AND PUBLISHERS ; TimerRepeater Class Objects
if __name__ == '__main__':
		global ackermann_ctrl
		rospy.init_node('e2o_ctrl')
		car_drive = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
		rospy.Subscriber('e2octrl', e2o_ctrl, e2o_ctrl_callback)
		e2o_pub = rospy.Publisher('e2ostatus', e2o_status, queue_size=1)
		rate = rospy.Rate(100)

		while not rospy.is_shutdown():

			car_drive.publish(ackermann_ctrl)

			rate.sleep()
		rospy.spin()
	
#=====================================================================
