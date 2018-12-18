#!/usr/bin/env python
import rospy
import time
import numpy as np
import threading
from PCANBasic import *
from e2o.msg import e2o_ctrl, e2o_status
from ackermann_msgs.msg import AckermannDrive

#from e2o.msg import e2o_ctrl, e2o_status

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

last_speed = 0
a1	= 0.2			# if input acceleration is "'a' then top speed that can be reached is a*a1
a2	= 0.02/50		# increment speed by a2*a
a3	= 0.005/50		# decrement speed by a3*a
b1	= 1/1500		# decrement speed by b1*brake %
c1	= 1/1000		# when gear is neutral, decrement speed by c1

# we know that the frequency is 100 

# SET OF ASSUMPTIONS

# let the car achieve a max. speed of 10 m/s for acceleration = 50 
# When acceleration is fixed to 50, the car takes 5 sec to reach 10 m/ss
# When acceleration is fixed and current speed is more than 'a*a1' then decrease speed at a rate of 'a2*a/4'
# Car takes 3 second to go down from 10 m/s to 0 m/s with brakes = 50
# When both accleration and brakes are 0, decrease speed by 1 m/s in every 10 seconds

flag = 0 # i is used so that no ackermann_ctrl is published until we get the first e2o_ctrl msg


m_PcanHandle = PCAN_USBBUS1
m_objPCANBasic = PCANBasic()

#=====================================================================
#DEFINE CONSTANTS

# Bit [55:51] Driving_Mode_RNDB_Cmd
# Bit [50] Driving mode RNDB Validity [Invalid =0| Valid=1]
# B6-> [55 54 53 52  51 1 0 0]
# 00001 -> Reverse 	
# 00010 -> neutral 	
# 00100 -> FwdDrive 
# 01000 -> Reserved
# 10000 -> Boost 
RNDB_NEUTRAL = 0x14
RNDB_REVERSE = 0x0C
RNDB_DRIVE = 0x24
RNDB_BOOST = 0x84

# Bit [63:60] Driving_Turn_Indicator_Cmd
# Bit [59] Driving_Turn_Indicator_Cmd_Valid
# 63 62 61 60 <- Bits
# 1000 -> Left Blink at 1 Hz 
# 0001 -> Right Blink at 1 Hz 
# 1001 -> Hazard Blink at 1 Hz
# 0000 -> Off
# Others -> Reserved
LAMP_OFF = 0x09
LAMP_RIND = 0x19
LAMP_LIND = 0x89
LAMP_HAZARD = 0x99

# Bit [58:57] Driving_HL_Cmd
# Bit [56] Driving_HL_Cmd_Vld
# 58 57 <- Bits
# 01 -> Low Beam 
# 10 -> High Beam 
# 00 -> Off
# 11 -> Reserved
LAMP_LBEAM = 0x0B
LAMP_HBEAM = 0x0D

# Bit [55:54] Brake_Light_Indicator_Cmd
# Bit [53] Brake_Light_Indicator_Cmd_Vld
# 55 54 <- Bits
# 01 -> Reserved
# 10 -> Reserved
# 00 -> Off
# 11 -> On
LAMP_BKLIGHT_B7 = 0x09
LAMP_BKLIGHT_B6_ON = 0xE0
LAMP_BKLIGHT_B6_OFF = 0x20

# Bit [63:58] Steering_Cntrl_Deg_Cmd 
# VALUE: 0 - 40 -> valid range in degrees

# Bit [57:56] Steering_Cntrl_Dir_Cmd
# 10 -> Turn Left
# 01 -> Turn Right 
# 00, 11 -> reserved

# Bit [55] Steering control command direction Validity 
# Bit [54] Steering control command degree Validity
# 0 -> Invalid 1 -> Valid
STEER_ZERO = 0x01
STEER_LEFT = 0xA2
STEER_RIGHT = 0xA1

BRAKE_ZERO = 0x01
BRAKE_FULL = 0xC9

ACCEL_ZERO = 0x01
ACCEL_FULL = 0xC9

WIPER_OFF = 0x20
WIPER_INT = 0x60
WIPER_HIGH = 0xA0
HORN_ON = 0x18
HORN_OFF = 0x08


#=====================================================================

def car_setter():

	global ackermann_ctrl,last_speed
	
	ackermann_ctrl.steering_angle = car_control.Steer * pi/180

	if abs(ackermann_ctrl.steering_angle) > 40*pi/180:
		
		ackermann_ctrl.steering_angle = np.sign(ackermann_ctrl.steering_angle) * 40*pi/180

	#at any point of time one of the brakes and acceleration is equal to zero
	if car_control.Accel > 0 and car_control.RNDB != 'N':
		
			#if abs(last_speed - a1*car_control.Accel) < a2*car_control.Accel:
			#	speed = a1*car_control.Accel

			if last_speed > a1*car_control.Accel:
				speed = last_speed - a3*car_control.Accel
			elif last_speed < a1*car_control.Accel:
				speed = last_speed + a2*car_control.Accel
			else:
				speed = last_speed

	
	elif car_control.Brake > 0:

		speed = last_speed - b1*car_control.Brake
		if speed < 0:
			speed = 0

	elif car_control.RNDB == 'N' or (car_control.Accel == 0 and car_control.Brake == 0) :
		speed = last_speed - c1
		if speed < 0:
			speed = 0

	last_speed = speed
	
	if car_control.RNDB == 'R':
		speed = speed * (-1)
	
	ackermann_ctrl.speed = speed

	car_drive.publish(ackermann_ctrl)
#=====================================================================
#CALLBACK Function of Threads : tmrRead and tmrWrite
def ReadMessages():
	global car_ctrl,flag
	
	
	try:
		Status_pcan = PCAN_ERROR_OK
		while (not (Status_pcan & PCAN_ERROR_QRCVEMPTY)):
			
			flag = 1
			Message = m_objPCANBasic.Read(m_PcanHandle)
			
			Status_pcan = Message[0]
			if Message[0] == PCAN_ERROR_OK:
				MSG_DATA = Message[1]
				msgTimeStamp = Message[2]
				
				Message_time = (msgTimeStamp.micros + 1000 * msgTimeStamp.millis + 0x100000000 * 1000 * msgTimeStamp.millis_overflow)
				print (Message_time)
				for i in range(MSG_DATA.LEN):
					print(format(MSG_DATA.DATA[i],'02x'), " ")
				
				MsgID = format(MSG_DATA.ID,'04x')
				DATA7 = format(MSG_DATA.DATA[7],'02x')
				DATA6 = format(MSG_DATA.DATA[6],'02x')
				
				#=======================Vehicle speed status============================
				if (MsgID == '076c'):
							
					car_speed = int(DATA7,16)
					#print "Speed of Car : ", car_speed, " Kmph \n"

					#if (car_speed == 0) and (gear != 0):
					#	count_cmd = count_cmd + 1
					#else:
					#	count_cmd = 0

					#gear_select()
					#pub_gear.publish(Float64(gear))
					#car_speed_ROS_Message = Float64(car_speed)
					#target_speed_ROS_Message = Float64(target_speed)

					#pub_State.publish(car_speed_ROS_Message)  
					#pub_Target.publish(target_speed_ROS_Message)
				
				#=======================Wiper and Horn==================================		
				elif (MsgID == '076e'):
					if(DATA7 == 'a0'):
						print "Wiper : ON ---- Horn : ON"
					elif(DATA7 == '20'):
						print "Wiper : OFF ---- Horn : ON"
					elif(DATA7 == '80'):
						print "Wiper : ON ---- Horn : OFF"
					elif(DATA7 == '00'):
						print "Wiper : OFF ---- Horn : OFF"
					print "\n"
				
				#=======================THROTTLE=====================
				elif (MsgID == '0771'):
					print "Throttle Percentage of Car : ", int(DATA7,16)/2, "\n"
					Accel = int(DATA7,16)
					Accel_val = (Accel>>2)
					car_ctrl.Accel = Accel_val
				#=======================BRAKES======================
				elif (MsgID == '0773'):
					print "Braking Percentage of Car : ", int(DATA7,16)/2, "\n"
					Brake = int(DATA7,16)
					Brake_val = (Brake>>2)
					car_ctrl.Brake = Brake_val
				#=======================Steering Angle and Direction=====================
				elif (MsgID == '0775'):
					Steer = int(DATA7,16)
					Steer_angle = (Steer>>2)
									   
					if((Steer&3) == 2):						  #Clockwise
						Steer_angle = Steer_angle*1               
					elif((Steer&3) == 1):					  #AntiClockwise
						Steer_angle = Steer_angle*(-1)

					car_ctrl.Steer = Steer_angle
					#steer_feedback = Steer_angle
					#pub_steer.publish(Int64(steer_feedback))
					print "Steering Angle : ",Steer_angle, "\n"

				#=======================Turn Indicators and Head Lights=================
				elif (MsgID == '0777'):
					if(DATA7[0] == '8'):
						print "Indicator Status : Left Indicator"
					elif(DATA7[0] == '1'):
						print "Indicator Status : Right Indicator"
					elif(DATA7[0] == '9'):
						print "Indicator Status : Hazard Indicator"
					elif(DATA7[0] == '0'):
						print "Indicator Status : OFF"
					print "\n"

					if(DATA7[1] == '4'):
						print "Head Lights : ON"
					elif(DATA7[1] == '0'):
						print "Head Lights : OFF"
					print "\n"

				#=======================DRIVING MODE and RNDB============================
				elif (MsgID == '0779'):
					if(DATA7 == '40'):
						print "Driving Mode : Manual"
					elif(DATA7 == '80'):
						print "Driving Mode : Autonomous"
					elif(DATA7 == 'c0'):
						print "Driving Mode : Homing in Progress"
					else:
						print "Driving Mode : Invalid"
					print "\n"

					if(DATA6 == '08'):
						drive_mode = 2
						print "Gear Status : Reverse"
						car_ctrl.RNDB = 'R'
					elif(DATA6 == '10'):
						drive_mode = 0
						print "Gear Status : Neutral"
						car_ctrl.RNDB = 'N'
					elif(DATA6 == '20'):
						drive_mode = 1
						print "Gear Status : Drive"
						car_ctrl.RNDB = 'D'
					elif(DATA6 == '80'):
						print "Gear Status : Boost"
						car_ctrl.RNDB = 'B'
					else:
						print "Gear Status : Invalid"
					print "\n"


			#car_pub.publish(car_ctrl) # These are published just to see whehter pcan is working properly or not
			
			car_setter()

	except KeyboardInterrupt:
		
		tmrRead.stop()
		m_objPCANBasic.Uninitialize(m_PcanHandle)
		raise SystemExit, 1

#=====================================================================
#CALLBACK Function of Threads : tmrRead and tmrWrite

#=====================================================================
class TimerRepeater(object):
	def __init__(self, name, interval, target, isUi, args=[], kwargs={}):
		"""
		Creates a timer.
		Parameters:
			name        name of the thread
			interval    interval in second between execution of target
			target      function that is called every 'interval' seconds
			args        non keyword-argument list for target function
			kwargs      keyword-argument list for target function
		"""
		# define thread and stopping thread event
		self._name = name
		self._thread = None
		self._event = None
		self._isUi = isUi
		# initialize target and its arguments
		self._target = target
		self._args = args
		self._kwargs = kwargs
		# initialize timer
		self._interval = interval
		self._bStarted = False

	def _run(self):
		"""
		Runs the thread that emulates the timer.
		Returns:
			None
		"""
		while not self._event.wait(self._interval):
			if self._isUi:
				# launch target in the context of the main loop
				root.after(1, self._target,*self._args, **self._kwargs)
			else:
				self._target(*self._args, **self._kwargs)

	# Starts the timer
	def start(self):
		# avoid multiple start calls
		if (self._thread == None):
			self._event = threading.Event()
			self._thread = threading.Thread(None, self._run, self._name)
			self._thread.start()

	# Stops the timer
	def stop(self):
		if (self._thread != None):
			self._event.set()
			self._thread = None

#=====================================================================

#=====================================================================
class e2oCAN ():
	def __init__(self):
		rospy.init_node('e2o_can_receive', anonymous=False)
		car_pub = rospy.Publisher('e2octrl', e2o_ctrl,queue_size = 1) 		#Published just to see the raw output that we are receiving from CAN
		car_drive = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
		rospy.loginfo("Press CTRL+c to Stop")
		rospy.on_shutdown(self.shutdown)
		rate = rospy.Rate(10)
		self.initCAN()

		while not rospy.is_shutdown():
			rate.sleep()

	def shutdown(self):
		rospy.loginfo("Stopping ...")
		global m_objPCANBasic
		global m_PcanHandle
		m_objPCANBasic.Uninitialize(m_PcanHandle)

	def initCAN(self):
		#initialize e2OCAN messages
		e2OCAN_RNDB = TPCANMsg()
		e2OCAN_LAMP = TPCANMsg()
		e2OCAN_STEER = TPCANMsg()
		e2OCAN_BRAKE = TPCANMsg()
		e2OCAN_ACCEL = TPCANMsg()
		e2OCAN_WIPHORN = TPCANMsg()

		for i in range(8):
			e2OCAN_RNDB.DATA[i] = 0x00
			e2OCAN_LAMP.DATA[i] = 0x00
			e2OCAN_STEER.DATA[i] = 0x00
			e2OCAN_BRAKE.DATA[i] = 0x00
			e2OCAN_ACCEL.DATA[i] = 0x00
			e2OCAN_WIPHORN.DATA[i] = 0x00

		# Motorola Byte Order (Big Endian)
		# Little Endian (64-bits): 	B7 B6 B5 B4 B3 B2 B1
		# Big Endian (64-bits): 	B0 B1 B3 B4 B5 B6 B7

		# Bits in Big Endian Byte Order
		# B6->[55 54 53 52  51 50 49 48] B7->[63 62 61 60  59 58 57 56]

		e2OCAN_RNDB.ID = 0x778
		e2OCAN_RNDB.MSGTYPE = PCAN_MESSAGE_STANDARD
		e2OCAN_RNDB.LEN = 8

		e2OCAN_LAMP.ID = 0x776
		e2OCAN_LAMP.MSGTYPE = PCAN_MESSAGE_STANDARD
		e2OCAN_LAMP.LEN = 8

		e2OCAN_STEER.ID = 0x774
		e2OCAN_STEER.MSGTYPE = PCAN_MESSAGE_STANDARD
		e2OCAN_STEER.LEN = 8

		e2OCAN_BRAKE.ID = 0x772
		e2OCAN_BRAKE.MSGTYPE = PCAN_MESSAGE_STANDARD
		e2OCAN_BRAKE.LEN = 8

		e2OCAN_ACCEL.ID = 0x770
		e2OCAN_ACCEL.MSGTYPE = PCAN_MESSAGE_STANDARD
		e2OCAN_ACCEL.LEN = 8

		e2OCAN_WIPHORN.ID = 0x76D
		e2OCAN_WIPHORN.MSGTYPE = PCAN_MESSAGE_STANDARD
		e2OCAN_WIPHORN.LEN = 8

		#=====================================================================
		#Construct PCAN bus Handle and Print BUS Parameters
		global m_PcanHandle
		global m_objPCANBasic
		Status = m_objPCANBasic.Initialize(m_PcanHandle, PCAN_BAUD_500K, 0, 0, 0)
		print(Status)

		print("PCAN_CHANNEL_CONDITION (Before) = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_CONDITION)))
		print("PCAN_DEVICE_NUMBER = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_DEVICE_NUMBER)))
		print("PCAN_API_VERSION= " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_API_VERSION)))
		print("PCAN_HARDWARE_NAME = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_HARDWARE_NAME)))
		print("PCAN_CHANNEL_CONDITION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_CONDITION)))
		print("PCAN_CHANNEL_VERSION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_VERSION)))
		print("PCAN_BITRATE_INFO = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BITRATE_INFO)))
		print("PCAN_BUSSPEED_NOMINAL = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BUSSPEED_NOMINAL)))
		print("PCAN_RECEIVE_STATUS = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_RECEIVE_STATUS)))
#=====================================================================


#=====================================================================
#ROS NODE : SUBSCRIBERS AND PUBLISHERS ; TimerRepeater Class Objects
if __name__ == '__main__':
	try:
		e2oCAN()
		tmrRead = TimerRepeater("tmrRead", 0.010, ReadMessages, False)
		tmrRead.start()

	except:
		rospy.loginfo("End of the program")

#===================================================================