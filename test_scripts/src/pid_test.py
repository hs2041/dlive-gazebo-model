#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
import numpy as np 

t = AckermannDrive()
t.steering_angle_velocity = 1
t.speed = 6
t.steering_angle = 0 
t.acceleration = 0
t.jerk = 0 


def control_callback(msg, twist_pub):
	
	#global t
	
	effort = msg.data
	effort = effort/2   # set the range of effort from -80 to +80
	if abs(effort) > 100:
		effort = np.sign(effort)*100
		t.speed = 2
	elif abs(effort) >40:
		t.speed = 6
	else:
		t.speed = 6

	if abs(effort) < 2.5:
		effort = np.sign(effort)
	else:
		effort = int(effort*40/100)
	t.steering_angle = (effort)*np.pi /180

	
	#print('angle: ',t.steering_angle)

	
if __name__ == '__main__':
	
	#global t
	rospy.init_node('testing_pid')
	
	twist_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
	rospy.Subscriber('control_effort', Float64, control_callback, twist_pub)
	
	r = rospy.Rate(100)

	while not rospy.is_shutdown():
		
		twist_pub.publish(t)
		r.sleep()
	rospy.spin()