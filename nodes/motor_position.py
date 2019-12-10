#!/usr/bin/env python

# Author: Gabrielle Conard, July 2019
# File: motor_position.py
# This python script takes the leg angles from inverse_kinematics...
# ...and finds the positions of the two motors required to achieve those angles.

#basics
import rospy
import sys
import roslib
roslib.load_manifest('odrive_ros')


import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import  Float64, Int32, Bool   #,Float64Stamped, Int32Stamped,
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose #PoseStamped
import std_srvs.srv

import time
from numpy import *
import traceback
import Queue   # might not be needed

# ##timer code
import time

# # Block 1
# run_once = 0
# while 1:
# 		if run_once ==0:
# 			timing_time0 = True
# 			wait_time0 = False
# 			T0_EN = timing_time0
# 			T0 = False
# 			Start_time0 = 0
# 			run_once = 1


# # --------------------TIMER_0--------------------
# B_time0 = wait_time0 and T0
# C_time0 = timing_time0 and T0
# D_time0 = timing_time0 and not T0

# wait_time0 = B_time0 or C_time0
# timing_time0 = D_time0

# if (wait_time0):
# 	Start_time0 = time.time()

# if(timing_time0):
# 	delta_t0 = time.time() - Start_time0

# else:
# 	delta_t0 = 0

# T0 = delta_t0 > 5
# #-----------------------------------------------

# # # Block 2

# # A = Timing and not T0_EN
# # B = Wait and T0
# # C = Timing and not T1
# # D = Timing and T1

# # # Block 3

# # Wait = A or N
# # Forward = B or C
# # Back = D or E
# # TLeft = F or G
# # TRight = H or I
# # SRight = J or K
# # SLeft = L or M

# # # Block 4

# ##timer code end



class MotorPosition:

	def __init__(self):
		#if you need parameters, use the following
		#self.mything = rospy.get_param('param_name',default_value)
		self.theta_f = rospy.get_param('~femur_angle', "/theta_f")
		self.theta_t = rospy.get_param('~tibia_angle', "/theta_t")
		self.position_command = rospy.get_param('~position_command', "/cmd_pos1")
		self.cal_t = rospy.get_param('~cal_t',"motor_pos1")
		self.cal_f = rospy.get_param('~cal_f',"motor_pos2")
		self.doStuffTrue = rospy.get_param('~motor_initiation',True)



		self.lim1low_topic   = rospy.get_param('~lim1low_topic', "odrive/odrive1_low_tib")
		self.lim2high_topic   = rospy.get_param('~lim2high_topic', "odrive/odrive1_high_fem")

		# self.lim1low_sub = rospy.Subscriber(self.lim1low_topic ,Bool,self.lim1lowcallback)
		# self.lim2high_sub = rospy.Subscriber(self.lim2high_topic ,Bool,self.lim2highcallback)


		#subscribe to theta_f and theta_t from inverse_kinematics
		# self.sub_F = rospy.Subscriber("/theta_f", Float64Stamped, femur_motor_callback)
		# self.sub_T = rospy.Subscriber("/theta_t", Float64Stamped, tibia_motor_callback)
		self.sub_F = rospy.Subscriber(self.theta_f, Float64, self.femur_motor_callback)
		self.sub_T = rospy.Subscriber(self.theta_t, Float64, self.tibia_motor_callback)
		# self.sub_cal_T = rospy.Subscriber(self.cal_t, Pose, self.tibia_calibration_callback)
		# self.sub_cal_F = rospy.Subscriber(self.cal_f, Pose, self.femur_calibration_callback)
		self.lim1low = False
		self.lim1high = False
		self.lim2low = False
		self.lim2high = False
		self.init_motor_f = 0
		self.init_motor_t = 0


		# actual position (counts) from /encoder_right topic (encoder position) published by odrive_node.py
		# self.sub_init_t = rospy.Subscriber("odrive/raw_odom/encoder_right", Int32, self.init_t_callback)
		# self.sub_init_f = rospy.Subscriber("odrive/raw_odom/encoder_left", Int32, self.init_f_callback)


		#publish motor positions
		# How do I get both in one publisher to output a Pose msg?
		# Do I set up a timer?
		# self.pub = rospy.Publisher("/cmd_pos", PoseStamped, queue_size = 1)
		self.pub = rospy.Publisher(self.position_command, Pose, queue_size = 1)
		# self.pup_cal_t = rospy.Publisher(self.cal_t, Pose, queue_size = 2)
		# self.pup_cal_f = rospy.Publisher(self.cal_f, Pose, queue_size = 2)


		# Set up a timed loop
		# rospy.Timer(rospy.Duration(0.01), self.timer_callback, oneshot=False)

		# Class variables
		# Measured Values:
		# All lengths are in inches and angles are in radians
		self.constraint_H = 4.047  # distance from hip to link connection (along hip constraint)
		self.link_H = 6.981 #7.047    	   # length of link (from link connection at constraint to ball nut)
		self.mount_H = 2.294       # distance from hip to closest femur ball screw mount (inside face)

		self.constraint_K = 3.739  # distance from knee to link connection (along knee constraint)
		self.link_K = 6.981 #7.047		   # length of link (from link connection at constraint to ball nut)
		self.mount_K = 2.177	   # distance from knee to closest knee ball screw mount (inside face)

		# Full length of both ball screws is 180 mm (7.087 in)...
		# ...but ball nuts can't really travel the full range.  
		# Otherwise, things will crash into each other (eg: the links will run into the encoder mounts).
		# These effective lengths were measured empirically, from the mount referenced above...
		# ...to the farthest distance from the mount.
		self.ball_screw_H = 5.875 #3.75#5
		self.ball_screw_K = 5.5625 #4.462 #4.6063

		# Ball nut limits: minimum is the distance from the joint for the respective leg segment...
		# ...to the zero position.
		# Maximum is the distance from the same joint to the farthest distance the ball nut can travel.
		self.min_H = 4.375
		self.max_H = self.ball_screw_H + self.mount_H
		self.min_K = 3.5
		self.max_K = self.ball_screw_K + self.mount_K

		# To be received from subscribed topics
		# From inverse kinematics...
		self.theta_f = None   # angle of femur, from femur ball screw to hip constraint
		self.theta_t = None	  # angle of tibia, from tibia ball screw to knee constraint

		self.init_motor_f = 0
		self.init_motor_t = 0

		# To be calculated...
		self.theta_HLN = None   # angle between the constraint (HL) and linkage rod (LN)
		self.theta_HNL = None	# angle between femur ball screw (HN) and link (NL)
		self.theta_KLN = None	# angle between knee constraint (KL) and link (LN)
		self.theta_KNL = None	# angle between tibia ball screw (KN) and link (NL)

		self.length_HBN = None	# distance between hip and the hip ball nut
		self.length_KBN = None	# distance between knee and the knee ball nut

		self.des_BN_f = None	# desired location of femur ball nut with respect to lower ball screw mount
		# self.last_BN_f = 0
		# self.delta_BN_f = None
		self.des_BN_t = None	# desired location of tibia ball nut with respect to lower ball screw mount
		# self.last_BN_t = 0
		# self.delta_BN_t = None

		self.des_pos_f = 0	# desired femur motor position (in encoder counts)
		# self.last_pos_f = 0
		self.delta_motor_f = None
		self.des_pos_t = 0	# desired tibia motor position (in encoder counts)
		# self.last_pos_t = 0
		self.delta_motor_t = None
		self.calibrated_t = False
		self.calibrated_f = False


		# Conversion
		self.mm_to_in = 1 / 25.4   # 1 in = 25.4 mm
		self.distance_to_motor_pos = (2.2114 / (self.mm_to_in * 5)) * (8192)   # 5 mm = 2.2114 rev = 2.2114 * 8192 counts
		self.rev_to_count = 8192
		self.deg_to_count = 8192 / 360   # 1 revolution = 360 degrees = 8192 counts

		self.lim1low = False
    
		self.lim2high = False

	def lim1lowcallback(self,data):

		self.lim1low = data.data
		if self.lim1low and not self.lim1low_old and self.calibrated_t is False:
			self.calibrated_t = True
			rospy.logwarn(data)
			self.des_BN_t = 3.25
			self.delta_motor_t = self.des_BN_t * self.distance_to_motor_pos
			self.des_pos_t = self.delta_motor_t
		self.lim1low_old = self.lim1low


	def lim2highcallback(self,data):
	
		self.lim2high = data.data
		if self.lim2high and not self.lim2high_old and self.calibrated_f is False:
			self.calibrated_f = True
			rospy.logwarn(data)
			self.des_BN_f = 3.75
			self.delta_motor_f = self.des_BN_f * self.distance_to_motor_pos
			self.des_pos_f = self.delta_motor_f
		self.lim2high_old = self.lim2high

	# def init_f_callback(self, data):
	# 	if(self.init_motor_f is None):
	# 		self.init_motor_f = data.data
	# 		self.last_pos_f = self.init_motor_f
	# 	else:
	# 		pass


	# def init_t_callback(self, data):
	# 	if(self.init_motor_t is None):
	# 		self.init_motor_t = data.data
	# 		self.last_pos_t = self.init_motor_t
	# 	else:
	# 		pass
	# def tibia_calibration_callback(self):
	# 	rospy.logwarn('self.calibrated:')
	# 	if (self.calibrated_t is False):
	# 		if self.lim1high and not self.lim1high_old:
	# 			self.des_BN_t = 3.25
	# 			self.delta_motor_t = self.des_BN_t * self.distance_to_motor_pos
	# 			self.des_pos_t = self.delta_motor_t
	# 			self.lim1high_old = self.lim1high

	# 		motor_pos1 = Pose()
	# 		motor_pos1.position.x = self.des_pos_t
	# 		motor_pos1.position.y = 0   # Order depends on ODrive set-up
	# 		motor_pos1.position.z = 0.0
	# 		motor_pos1.orientation.x = 0.0
	# 		motor_pos1.orientation.y = 0.0
	# 		motor_pos1.orientation.z = 0.0
	# 		motor_pos1.orientation.w = 0.0
	# 		self.pub.publish(motor_pos1)
	# 		print("Motor Positions: " + str(motor_pos))
	# 		self.calibrated_t = True

	# def femur_calibration_callback(self):
	# 	if (self.calibrated_f is False):
	# 		if self.lim2low and not self.lim2low_old:
	# 			self.des_BN_f = 3.75
	# 			self.delta_motor_f = self.des_BN_f * self.distance_to_motor_pos
	# 			self.des_pos_f = self.delta_motor_f
	# 			self.lim2low_old = self.lim2low

	# 		motor_pos2 = Pose()
	# 		motor_pos2.position.x = 0
	# 		motor_pos2.position.y = self.des_pos_f   # Order depends on ODrive set-up
	# 		motor_pos2.position.z = 0.0
	# 		motor_pos2.orientation.x = 0.0
	# 		motor_pos2.orientation.y = 0.0
	# 		motor_pos2.orientation.z = 0.0
	# 		motor_pos2.orientation.w = 0.0
	# 		self.pub.publish(motor_pos2)
	# 		print("Motor Positions: " + str(motor_pos))
	# 		self.calibrated_f = True

	def femur_motor_callback(self, data):
		# Femur angle is calculated from Inverse Kinematics code, zero is when the hip is tucked 
		self.theta_f = data.data 
		print(self.theta_f)

		# Calculations will not continue if no init_motor_f does not have a value 
		if(self.init_motor_t is not None and self.calibrated_f is True):
			# rospy.logwarn('femur_motor_callback')
			# Calculations will not continue if theta_f does not have a value
			if(self.theta_f is not None):
				print("Received theta_f!")
				
				# Calculates the angle between ball screw and link
				self.theta_HNL = arcsin((self.constraint_H * sin(self.theta_f)) / self.link_H)
				# Calculates the angle between the constraint and linkage rod
				self.theta_HLN = pi - self.theta_f - self.theta_HNL

				#length from hip to ball nut
				self.length_HBN = ((self.link_H * sin(self.theta_HLN)) / sin(self.theta_f))
				print("HBN = "+ str(self.length_HBN)) 

				# Check: Make sure the ball nut will not crash into either ball screw mount
				if(self.length_HBN < self.min_H):
					# If less than minimum spacing between ball nut and H, 
					# ball nut will crash into upper ball screw mount...
					# ...so reset value to maximum ball nut position (relative to lower mount.)
					print("Femur ball nut can't go that far! Resetting to maximum position...")
					self.length_HBN = self.min_H   
				elif(self.length_HBN > (self.ball_screw_H + self.mount_H)):
					# If less than maximum spacing between ball nut and H, 
					# ball nut will crash into lower ball screw mount...
					# ...so reset value to minimum ball nut position (relative to lower mount).
					print("Femur ball nut can't go that far! Resetting to minimum position...")
					self.length_HBN = self.ball_screw_H + self.mount_H
				else:
					# If in between these two extremes, do nothing.  Everything should be fine!
					pass


				# desired change in length (should be positive) of the ball screw
				# Take the distance between the knee and the nearest ball screw mount into account...
				# ...to find desired ball nut location, as well as the total length of the screw
				self.des_BN_f = self.ball_screw_H + self.mount_H - (self.length_HBN)
				print("BN_f = "+ str(self.des_BN_f))
				

				# # Check: Make sure the ball nut will not crash into either ball screw mount
				# if(self.des_BN_f < 0):
				# 	# If negative, ball nut will crash into lower ball screw mount...
				# 	# ...so reset value to minimum ball nut position.
				# 	print("Femur ball nut can't go that far! Resetting to minimum position...")
				# 	self.des_BN_f = 0
				# elif(self.des_BN_f > self.ball_screw_H):
				# 	# If larger than maximum ball screw length, ball nut will crash into upper ball screw mount...
				# 	# ...so reset value to maximum ball nut position
				# 	print("Femur ball nut can't go that far! Resetting to maximum position...")
				# 	self.des_BN_f = self.ball_screw_H
				# else:
				# 	# If positive and less than max, do nothing. Everything should be fine!
				# 	pass


				#
				#self.delta_BN_f = self.des_BN_f - self.last_BN_f
				#print(self.delta_BN_f)

				# Reset value of last ball nut positon
				# Do we want real feedback?
				#self.last_BN_f = self.des_BN_f

				# Finds motor position based on ball nut position
				self.delta_motor_f = - self.des_BN_f * self.distance_to_motor_pos

				self.des_pos_f = self.delta_motor_f #+ self.last_pos_f
				

			else:
				pass
		elif(self.init_motor_f is not None and self.calibrated_f is False and self.doStuffTrue is True):
				self.des_pos_f=self.des_pos_f-10
				# rospy.logwarn('elif_femur')

		else:
			pass

	def tibia_motor_callback(self, data):
		# rospy.logwarn('entering tibia motor callback')
		# Calculations will not continue if no init_motor_f does not have a value 
		if(self.init_motor_t is not None and( self.calibrated_t is True and self.calibrated_f is True)):
			# Calculations will not continue if theta_f does not have a value
			if(self.theta_t is not None):
				print("Received theta_f and theta_t!")
				self.theta_t = data.data
				print("theta_t = " + str(self.theta_t))

				# Calculate the angle between tibia ball screw (KN) and link (NL)
				self.theta_KNL = arcsin((self.constraint_K * sin(self.theta_t)) / self.link_K)
				print("theta_KNL = " + str(self.theta_KNL))
				# Calculate the angle between knee constraint (KL) and link (LN)
				self.theta_KLN = pi - self.theta_KNL - self.theta_t

				# Adding offset angles (original is above)
				# self.theta_KLN = pi - self.theta_KNL - self.theta_t - self.theta_t_shift
				# print("theta_KLN = " + str(self.theta_KNL))

				# Calculate the distance between the knee and the ball nut
				self.length_KBN = (self.link_K * sin(self.theta_KLN)) / sin(self.theta_t)
				print("length_KBN = " + str(self.length_KBN))

				# Check: Make sure the ball nut will not crash into either ball screw mount
				if(self.length_KBN < self.min_K):
					# If less than minimum spacing between ball nut and K, 
					# ball nut will crash into lower ball screw mount...
					# ...so reset value to minimum ball nut position (relative to lower mount).
					print("Tibia ball nut can't go that far! Resetting to minimum position...")
					self.length_KBN = self.min_K
				elif(self.length_KBN > (self.ball_screw_K + self.mount_K)):
					# If less than maximum spacing between ball nut and K, 
					# ball nut will crash into upper ball screw mount...
					# ...so reset value to maximum ball nut position (relative to lower mount).
					print("Tibia ball nut can't go that far! Resetting to maximum position...")
					self.length_KBN = self.ball_screw_K + self.mount_K
				else:
					# If in between these two extremes, do nothing.  Everything should be fine!
					pass

				# self.des_BN_t = self.ball_screw_K - self.length_KBN
				# Take the distance between the knee and the nearest ball screw mount into account...
				# ...to find desired ball nut location
				#self.des_BN_t = self.length_KBN - self.mount_K  # original line
				self.des_BN_t = self.length_KBN - self.min_K
				print("des_BN_t = " + str(self.des_BN_t))
				#self.delta_BN_t = self.des_BN_t - self.last_BN_t
				#print("delta_BN_t = " + str(self.delta_BN_t))

				# Reset value of last ball nut positon
				# Do we want real feedback?
				#self.last_BN_t = self.des_BN_t


				# # Check: Make sure the ball nut will not crash into either ball screw mount
				# if(self.des_BN_t < 0):
				# 	# If negative, ball nut will crash into lower ball screw mount...
				# 	# ...so reset value to minimum ball nut position.
				# 	print("Tibia ball nut can't go that far! Resetting to minimum position...")
				# 	self.des_BN_t = 0
				# elif(self.des_BN_t > self.ball_screw_K):
				# 	# If larger than maximum ball screw length, ball nut will crash into upper ball screw mount...
				# 	# ...so reset value to maximum ball nut position
				# 	print("Tibia ball nut can't go that far! Resetting to maximum position...")
				# 	self.des_BN_t = self.ball_screw_K
				# else:
				# 	# Do nothing
				# 	pass

				
				# Finds motor position based on ball nut position
				self.delta_motor_t = self.des_BN_t * self.distance_to_motor_pos
				# print("delta_motor_t = " + str(self.delta_motor_t))
				self.des_pos_t = self.delta_motor_t #+ self.last_pos_t
		
				# Publishes the motor positions as a Pose message
				# motor_pos = PoseStamped()
				# motor_pos.header.stamp = rospy.Time.now()
				motor_pos = Pose()
				motor_pos.position.x = self.des_pos_t
				motor_pos.position.y = self.des_pos_f   # Order depends on ODrive set-up
				motor_pos.position.z = 0.0
				motor_pos.orientation.x = 0.0
				motor_pos.orientation.y = 0.0
				motor_pos.orientation.z = 0.0
				motor_pos.orientation.w = 0.0
				self.pub.publish(motor_pos)
				print("Motor Positions: " + str(motor_pos))

				# Reset value of last motor positons
				# Do we want real feedback?
				# self.last_pos_f = self.des_pos_f
				# self.last_pos_t = self.des_pos_t
			else:
				pass
		elif(self.init_motor_t is not None and (self.calibrated_t is False or self.calibrated is False) and self.doStuffTrue is True):
			self.des_pos_t=self.des_pos_t-10 
			motor_pos = Pose()
			motor_pos.position.x = self.des_pos_t
			motor_pos.position.y = self.des_pos_f   # Order depends on ODrive set-up
			motor_pos.position.z = 0.0
			motor_pos.orientation.x = 0.0
			motor_pos.orientation.y = 0.0
			motor_pos.orientation.z = 0.0
			motor_pos.orientation.w = 0.0
			self.pub.publish(motor_pos)
			print("Motor Positions: " + str(motor_pos))
			# rospy.logwarn('elif_tibia')

		else:
			pass



	# function that takes these two distances and converts them into motor positions
	# need a self.last_pos_f too so that motor knows where it needs to turn to
	# consider direction (cw or ccw)
	# def timer_callback(self, data):
	# 	if(self.delta_BN_f and self.delta_BN_t is not None):
	# 		self.delta_motor_f = self.delta_BN_f * self.distance_to_motor_pos
	# 		self.des_pos_f = self.delta_motor_f + self.last_pos_f

	# 		self.delta_motor_t = self.delta_BN_t * self.distance_to_motor_pos
	# 		self.des_pos_t = self.delta_motor_t + self.last_pos_t

	# 		motor_pos = PoseStamped()
	# 		header.stamp = rospy.Time.now()
	# 		motor_pos = (self.des_pos_f, self.des_pos_t)
	# 		self.pub.publish(motor_pos)

	# 		# Reset value of last motor positons
	# 		# Do we want real feedback?
	# 		self.last_pos_f = self.des_pos_f
	# 		self.last_pos_t = self.des_pos_t
	# 	else:
	# 		pass




def main(args):
	rospy.init_node('motor_position',anonymous=True)
	MP = MotorPosition()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
