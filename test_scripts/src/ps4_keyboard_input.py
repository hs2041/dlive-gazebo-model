#!/usr/bin/env python
import rospy
import time
import pyautogui
from math import pi
from sensor_msgs.msg import Joy

t1 = time.time()
t2 = time.time()

pyautogui.press("a")
prev_press = 'o'

set_array = ['a','b','c','d','e','f','g','h','i','j','k','l','m','n']

def ps4_callback(data):
	#global car_control
  #car_control.Steer = int(round(data.axes[0]*180/pi, 4))

  global prev_press,set_array,t1,t2	
  #print(pyautogui.size())
  t2 = time.time()
  if t2 - t1 >0.5:
  	prev_press = 'o'
  l,b = pyautogui.size()
  x,y = pyautogui.position()
  '''
  check = data.axes[11]
  if check == 1:
 	  x1 = data.axes[0]
	  y1 = data.axes[1]
	  
	  pyautogui.moveTo(int(x-10*x1), int(y-10*y1))
  '''

  for i in range(14):
  	if data.buttons[i] == 1 :
  		if set_array[i] == prev_press:
  			pass
  		else:
  			pyautogui.press(set_array[i])
  			#print(set_array[i])
  			t1 = time.time()
  			prev_press = set_array[i]
  			print(" next line ")



if __name__ == '__main__':
    rospy.init_node('ps4_e2o_ctrl', anonymous=True)
    rospy.Subscriber("joy", Joy, ps4_callback)
    rospy.spin()