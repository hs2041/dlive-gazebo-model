#!/usr/bin/env python
import rospy
import time
import threading
import math
from PCANBasic import *
from e2o.msg import e2o_ctrl, e2o_status

#INITIALISE VARIABLES
max_steer_angle_radian = 0.69              # Based on E20 turning radius and allowed steering angle, max value will be decided, beyond which steering... 
min_steer_angle_radian = -0.69				# ...angle will get clamped to maximum; theoretically, inner wheel turns by 39 degree and outer by 29 degrees 

car_speed = 0							#Current_car_speed (Kmph)
target_speed = 0						#Target speed (Kmph)
target_steer = 0						#Target Steering angle (With sign in Degrees)
steer_feedback = 0						#Steering angle Feedback (With sign in Degrees)
count_cmd = 0 							#Number of speedometer reads between two cmd_vel_receives
vx_target = 0 							#Velocity_x (with sign) - Target
drive_mode = 0							#Driving Mode (R = 2; N = 0; D = 1)
gear_neutral = True 					#Is(Gear == NEUTRAL)
new_cmd_vel =  False 					#Whether a new command has been received for the velocity
Initial_Brake = 0						#Braking at Initial neutral condition = 30%
gear = 0
acceleration = 0
brake = 0

car_status = e2o_status()

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
HORN_OFF = 0xE8

#=====================================================================
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
m_PcanHandle = PCAN_USBBUS1
m_objPCANBasic = PCANBasic()
Status = m_objPCANBasic.Initialize(m_PcanHandle, PCAN_BAUD_500K, 0, 0, 0)

print(Status)
print("PCAN_CHANNEL_CONDITION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_CONDITION)))
print("PCAN_DEVICE_NUMBER = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_DEVICE_NUMBER)))
print("PCAN_API_VERSION= " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_API_VERSION)))
print("PCAN_HARDWARE_NAME = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_HARDWARE_NAME)))
print("PCAN_CHANNEL_VERSION = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_CHANNEL_VERSION)))
print("PCAN_BITRATE_INFO = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BITRATE_INFO)))
print("PCAN_BUSSPEED_NOMINAL = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_BUSSPEED_NOMINAL)))
print("PCAN_RECEIVE_STATUS = " + str(m_objPCANBasic.GetValue(m_PcanHandle, PCAN_RECEIVE_STATUS)))

#Set the Initial Parameters of the Car
e2OCAN_LAMP.DATA[6] = 0x20
e2OCAN_STEER.DATA[6] = 0xC0
e2OCAN_RNDB.DATA[6] = RNDB_NEUTRAL
e2OCAN_LAMP.DATA[7] = LAMP_HAZARD
e2OCAN_STEER.DATA[7] = STEER_ZERO
e2OCAN_BRAKE.DATA[7] = ((Initial_Brake/2)<<1) | 1
e2OCAN_ACCEL.DATA[7] = ACCEL_ZERO
e2OCAN_WIPHORN.DATA[7] = WIPER_OFF + 0x08


def WriteMessages():
	try:
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_RNDB)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_LAMP)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_STEER)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_BRAKE)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_ACCEL)
		m_objPCANBasic.Write(m_PcanHandle, e2OCAN_WIPHORN)
		
	except KeyboardInterrupt:
		tmrWrite.stop()
		m_objPCANBasic.Uninitialize(m_PcanHandle)
		raise SystemExit, 1

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

def e2o_ctrl_callback(ctrl_data):
	#print("Inside callback")
	if ctrl_data.RNDB == "N":
		#print("Neutral Selected")
		e2OCAN_RNDB.DATA[6] = RNDB_NEUTRAL
	if ctrl_data.RNDB == "R":
		#print("Reverse Selected")
		e2OCAN_RNDB.DATA[6] = RNDB_REVERSE
	if ctrl_data.RNDB == "D":
		#print("Forward Selected")
		e2OCAN_RNDB.DATA[6] = RNDB_DRIVE
	if ctrl_data.RNDB == "B":
		#print("Boost Selected")
		e2OCAN_RNDB.DATA[6] = RNDB_BOOST

	if ctrl_data.Steer >= 0:
		e2OCAN_STEER.DATA[7] = ((abs(ctrl_data.Steer)<<2)| 2)
	else:
		e2OCAN_STEER.DATA[7] = ((abs(ctrl_data.Steer)<<2)| 1)

	if ctrl_data.Accel > 30:
		ctrl_data.Accel = 30

	e2OCAN_ACCEL.DATA[7]= (ctrl_data.Accel<<1) | 1
	e2OCAN_BRAKE.DATA[7]= (ctrl_data.Brake<<1) | 1

	if ctrl_data.Horn == 1:
		e2OCAN_WIPHORN.DATA[7] |= HORN_ON
	else:
		e2OCAN_WIPHORN.DATA[7] &= HORN_OFF

#=====================================================================
#ROS NODE : SUBSCRIBERS AND PUBLISHERS ; TimerRepeater Class Objects
if __name__ == '__main__':
	try:
		
		rospy.init_node('e2o_ctrl')
		
		tmrWrite = TimerRepeater("tmrWrite", 0.010, WriteMessages, False)
		tmrWrite.start()
		while not rospy.is_shutdown():
			rospy.spin()
	except:
		rospy.loginfo("End of the program")

#=====================================================================
