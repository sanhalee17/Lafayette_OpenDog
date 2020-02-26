#!/usr/bin/env python

# Author: Gabrielle Conard, July 2019
# File: inverse_kinematics.py
# This python script takes a given position of the foot (with respect to the hip)...
# ...and finds the angles of the upper and lower leg (femur and tibia) required to achieve that position.

#basics
import rospy
import sys
import roslib
roslib.load_manifest('odrive_ros')


import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import Float64  #,Float64Stamped, Int32Stamped
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose, PoseStamped
import std_srvs.srv

import time
from numpy import *
import traceback
import Queue   # might not be needed


class InverseKinematics:
	def __init__(self):
				# Class Variables
		# Note: Origin is at hip joint
		# For right leg: positive x is to the right, y is down, and positive angle is clockwise

		# Measured Values:
		# All lengths are in inches and angles are in degrees (and converted to radians)
		self.length_f = 14.125   # distance from hip to knee pivots
		self.length_t = 13.25    # distance from knee pivots to bottom of foot (P)
		self.theta_K_shift = 13*(pi/180)   #16.6# offset angle, between hip-knee (HK) line and hip-femur ball nut (HN) line
		self.theta_HKP_shift = -1*(pi/180)    #8-1.5#1# offset angle, between knee-foot (KP) line and knee-tibia link connection (KL) line
		self.theta_H = 10.3*(pi/180)         #10.3 #15.4 # 16.9 # offset angle, between x-axis and hip constraint (HL)
		self.theta_t_shift = 14*(pi/180)   #21.3-7.3 #15.8# offset angle, between knee-tibia ball nut (KN) line and knee-hip (KH) line
		#hip realted values, added by SL Feb.21.2020
		self.length_h = 4.5      #inches, from the center of leg(from the front view) to the center of hip bearing
		self.length_hbrv = 5     #inches, vertical length from hip bearing to actuator rod joint
		self.length_rte = 1      #inches, horizontal length from actuator rod joint to left end of 40*40 extrusion


		# Range of Motion
		# self.R = 27.6236  # Farthest reach of foot (P) relative to hip, empirical
		self.R = 27.375   	# Farthest reach of foot (P) relative to hip, sum of length_f and length_t
		self.r = 17.9904    # Closest reach of foot (P) relative to hip, empirical
		self.theta_min_E = 39.91*(pi/180)     # smallest angle from x-axis to hip-foot (HP) line when tibia is fully extended
		self.theta_max_E = 124*(pi/180) #119.29   # largest angle from x-axis to HP when tibia is fully extended
		self.theta_min_C = 4.0002*(pi/180)	  # smallest angle from x-axis to HP when tibia is fully contracted (folded)
		self.theta_max_C = 71.1922*(pi/180)   # largest angle from x-axis to HP when tibia is fully contracted (folded)

		#here, need to add hip range of motion
		self.theta_min_H = 1111     #something change this value in the future
		self.theta_max_H = 111111   #also change this

		#if you need parameters, use the following
		#self.mything = rospy.get_param('param_name',default_value)

		self.theta_f = rospy.get_param('~femur_angle', "/theta_f")
		self.theta_t = rospy.get_param('~tibia_angle', "/theta_t")
		self.theta_h = rospy.get_param('~hip_angle',"/theta_h")
		self.foot_position = rospy.get_param('~foot_position', "/footPosition_1")

		# Publishers and Subscribers
		# Subscribe to a foot position P: (xP, yP)
		# self.sub = rospy.Subscriber("/footPosition",Pose, self.pos_callback)
		self.sub = rospy.Subscriber(self.foot_position,PoseStamped, self.pos_callback)

		#publish leg angles (two separate publishers)
		#do I need to publish on a timer or only when I get a new value???
		# self.femur = rospy.Publisher("/theta_f", Float64Stamped, queue_size = 1)
		# self.tibia = rospy.Publisher("/theta_t",Float64Stamped, queue_size = 1)
		self.femur = rospy.Publisher(self.theta_f, Float64, queue_size = 1)
		self.tibia = rospy.Publisher(self.theta_t, Float64, queue_size = 1)
		self.hip = rospy.Publisher(self.theta_h, Float64, queue_size = 1)





		# To be calculated...
		self.d = None          # distance from hip to base of foot (or P)
		self.theta_P = None    # angle from foot position to x-axis (positive angle, towards negative x-axis)
		self.theta_K = None    # angle from foot (P) to knee pivot (between the lines HP and HK)
		self.theta_HKP = None  # angle from femur (upper leg) to tibia (lower leg) (between lines HK and KP)

		#added by SL Feb.21.2020
		self.h_d = None        # distance from hip bearing center to a foot
		self.theta_hij = None  # angle from hip bearing center to hip actuator rod joint

		# Goal:
		self.theta_f = None    # angle of femur, from femur ball screw to hip constraint
		self.theta_t = None    # angle of tibia, from tibia ball screw to knee constraint
		self.theta_h = None	   # angle of hip, from


	def pos_callback(self, data):
		print("Received position!")
		# Calculate distance from hip joint to foot (d)...
		# ...and angle with respect to (negative) x-axis
		self.d = sqrt(data.pose.position.x**2 + data.pose.position.y**2)
		self.theta_P = pi - arctan2(data.pose.position.y, data.pose.position.x)
		print(self.theta_P)

		# Check to see if the point and angle are within the allowed range of motion
		if(self.d > self.R):
			print("Too far, can't reach that!")
		elif(self.d < self.r):
			print("Too close, can't reach that either")
		# elif(self.theta_P > self.theta_max_E):
		# 	print("Angle is too large")
		# elif(self.theta_P < self.theta_min_C):
		# 	print("Angle is too small")
		else:
			# Compute angles foot-hip-knee (theta_K) and hip-knee-foot (theta_HKP)
			self.theta_K = arccos((self.length_f**2 + self.d**2 - self.length_t**2)/(2 * self.d * self.length_f))
			self.theta_HKP = arccos((self.length_t**2 + self.length_f**2 - self.d**2) / (2 * self.length_f * self.length_t))

			# Calculate desired angle of femur (taking offsets into account)...
			# ...and publish results
			# self.theta_f = Float64Stamped()
			# self.theta_f.header.stamp = rospy.Time.now()
			self.theta_f = Float64()
			self.theta_f = self.theta_P - self.theta_K - self.theta_K_shift + self.theta_H
			print("theta_f: " + str(self.theta_f))
			self.femur.publish(self.theta_f)

			# Calculate desired angle of tibia (taking offsets into account)...
			# ...and publish results
			# self.theta_t = Float64Stamped()
			# self.theta_t.header.stamp = rospy.Time.now()
			self.theta_t = Float64()
			self.theta_t = self.theta_HKP - self.theta_HKP_shift - self.theta_t_shift
			print("theta_t: " + str(self.theta_t))
			self.tibia.publish(self.theta_t)

			#calculate desired angle of hip (nothing is calculated here as of 12/3/19. This is here to publish a dummy topic to use for developing the hip node)
			self.theta_h = Float64()
			self.hip.publish(self.theta_h)
			print("theta_t: " + str(self.theta_h))
			#rospy.logwarn(str(self.theta_t) + ', ' + str(self.theta_f))





def main(args):
	rospy.init_node('inverse_kinematics',anonymous=True)
	IK = InverseKinematics()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
