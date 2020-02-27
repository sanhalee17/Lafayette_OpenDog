#!/usr/bin/env python


#basics
import rospy
import sys
import roslib
roslib.load_manifest('odrive_ros')


import tf.transformations
import tf_conversions
import tf2_ros
import math

# Imports message types and services from several libraries
from std_msgs.msg import  Float64, Int32, Bool   #,Float64Stamped, Int32Stamped,
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose #PoseStamped
import std_srvs.srv

import time
from numpy import *
import traceback
import Queue   # might not be needed
class HipPosition:
	def __init__(self):
		self.theta1_h = rospy.get_param('~hip1_angle', "/theta_h_1")
		self.theta2_h = rospy.get_param('~hip2_angle', "/theta_h_2")
		self.position_command = rospy.get_param('~position_command', "/cmd_pos5")


		self.sub1_H = rospy.Subscriber(self.theta1_h, Float64, self.hip1_pos_callback)
		self.sub2_H = rospy.Subscriber(self.theta2_h, Float64, self.hip2_pos_callback)


		self.pub = rospy.Publisher(self.position_command, Pose, queue_size = 1)
		self.init_motor_h = 0
		self.hip_link_dist = 5.625 #distance from hip joint to link in inches
		self.rev_to_count = 8192
		self.deg_to_count = 8192 / 360   # 1 revolution = 360 degrees = 8192 counts
		self.bs_l = 0 #length of ball screw actuator. starts at 0
		self.bs_l_per_motor_rotate = 0.00256 #amount the ball screw
		self.max_H = 0.02141
		self.min_H = -0.04642


	def hip1_pos_callback(self, data):

			self.theta1_h = data.data -90
			print(self.theta1_h)


			if(self.init_motor_h is not None):
				# Calculations will not continue if theta_f does not have a value
				if(self.theta1_h is not None):
					print("Received theta_h!")
					self.bs_l = math.sin(self.theta1_h) * self.hip_link_dist

					# Check: Make sure the ball nut will not crash into either ball screw mount
					if(self.bs_l < self.min_H):
						# If less than minimum spacing between ball nut and H,
						# ball nut will crash into upper ball screw mount...
						# ...so reset value to maximum ball nut position (relative to lower mount.)
						print("hip ball nut can't go that far! Resetting to maximum position...")
						self.bs_l= self.min_H
					elif(self.bs_l > self.max_H):
						# If less than maximum spacing between ball nut and H,
						# ball nut will crash into lower ball screw mount...
						# ...so reset value to minimum ball nut position (relative to lower mount).
						print("hip ball nut can't go that far! Resetting to minimum position...")
						self.bs_l = self.max_H
					else:
						# If in between these two extremes, do nothing.  Everything should be fine!
						pass


					self.des_pos1_h = self.bs_l/self.bs_l_per_motor_rotate * self.rev_to_count
 					hip_pos1 = Pose()
					hip_pos1.position.x = 0.0
					hip_pos1.position.y = 0.0
					hip_pos1.position.z = self.des_pos1_h
					hip_pos1.orientation.x = 0.0
					hip_pos1.orientation.y = 0.0
					hip_pos1.orientation.z = 0.0
					hip_pos1.orientation.w = 0.0
					self.pub.publish(hip_pos1)


					pass
			else:
				pass
	def hip2_pos_callback(self, data):
			# Femur angle is calculated from Inverse Kinematics code, zero is when the hip is tucked
			self.theta2_h = data.data
			print(self.theta2_h)

			# Calculations will not continue if no init_motor_f does not have a value
			if(self.init_motor_h is not None):
				# Calculations will not continue if theta_f does not have a value
				if(self.theta2_h is not None):
					print("Received theta_h!")


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




					self.delta_motor_h = - self.des_BN_f * self.distance_to_motor_pos

					self.des_pos2_h = self.delta_motor_f #+ self.last_pos_f
					hip.pos2 = Pose()
					hip_pos2.position.x = 0.0
					hip_pos2.position.y = 0.0
					hip_pos2.position.z = self.des_pos2_h
					hip_pos2.orientation.x = 0.0
					hip_pos2.orientation.y = 0.0
					hip_pos2.orientation.z = 0.0
					hip_pos2.orientation.w = 0.0
					self.pub.publish(hip_pos2)


					pass
			else:
				pass
def main(args):
	rospy.init_node('hip_position',anonymous=True)
	HP = HipPosition()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
