#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
import numpy as np 

t = AckermannDrive()
t.steering_angle_velocity = 1
t.speed = 0 
t.steering_angle = 0 
t.acceleration = 0
t.jerk = 0 

key_mapping = { 'w': [ 0, 1], 'x': [0, -1],
'a': [-1, 0], 'd': [1, 0],
's': [ 0, 0] }

def keys_cb(msg, twist_pub):
	
	if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
		return # unknown key
	vels = key_mapping[msg.data[0]]

	if msg.data[0] == 'w':	
		t.speed += 0.05 

	elif msg.data[0] == 's':
		t.speed -= 0.05 
	
	elif msg.data[0] == 'd':
		t.steering_angle -= 0.05
	
	elif msg.data[0] == 'a':
		t.steering_angle += 0.05
	
	if abs(t.speed)> 5.5: # 5.5 m/s is nearly equal to 20 km/hr
		t.speed = np.sign(t.speed) * 5.5

	if abs(t.steering_angle) < 0.04:
		t.steering_angle = 0

	if abs(t.speed) < 0.04:
		t.speed = 0

	if abs(t.steering_angle) > np.pi*40/180:
		t.steering_angle = np.sign(t.steering_angle) * np.pi *40/180
	#print('angle: ',t.steering_angle)

	
if __name__ == '__main__':
	rospy.init_node('keys_to_twist')
	twist_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
	rospy.Subscriber('keys', String, keys_cb, twist_pub)
	
	r = rospy.Rate(30)

	while not rospy.is_shutdown():
		
		if abs(t.speed)> 0.02:
			t.speed = t.speed - np.sign(t.speed) * 0.01
		if abs(t.steering_angle) >0.02:
			t.steering_angle = t.steering_angle - np.sign(t.steering_angle)*0.01
		twist_pub.publish(t)
		r.sleep()
	rospy.spin()