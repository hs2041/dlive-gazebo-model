#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive

t = AckermannDrive()
t.steering_angle_velocity = 1

key_mapping = { 'w': [ 0, 1], 'x': [0, -1],
'a': [-1, 0], 'd': [1, 0],
's': [ 0, 0] }

def keys_cb(msg, twist_pub):
	if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
		return # unknown key
	vels = key_mapping[msg.data[0]]

	if msg.data[0] == 'w':
		if t.speed < 0:
			t.speed = 0
		else:
			t.speed += 0.05 
	elif msg.data[0] == 's':
		if t.speed > 0:
			t.speed = 0
		else:
			t.speed -= 0.05 
	if msg.data[0] == 'a':
		if t.steering_angle < -1:
			pass
		elif t.steering_angle > 0:
			t.steering_angle = 0
		else:
			t.steering_angle -= 0.05 
	elif msg.data[0] == 'd':
		if t.steering_angle > 1:
			pass
		elif t.steering_angle < 0:
			t.steering_angle = 0
		else:
			t.steering_angle += 0.05
	#print('angle: ',t.steering_angle)

	twist_pub.publish(t)

if __name__ == '__main__':
	rospy.init_node('keys_to_twist')
	twist_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
	rospy.Subscriber('keys', String, keys_cb, twist_pub)
	rospy.spin()