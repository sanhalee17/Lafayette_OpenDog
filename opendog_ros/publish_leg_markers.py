#!/usr/bin/env python

# Author: Gabrielle Conard, July 2019
# File: motor_position.py
# This python script takes the leg angles from inverse_kinematics...
# ...and finds the positions of the two motors required to achieve those angles.

#basics
import rospy
import sys
import roslib
roslib.load_manifest('opendog_ros')

import tf
import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import  Float64, Int32   #,Float64Stamped, Int32Stamped,
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose #PoseStamped
import std_srvs.srv
from visualization_msgs.msg import Marker

import time
from numpy import *
import traceback
import Queue   # might not be needed


class MotorPosition:
	def __init__(self):
		
		# Class variables
		self.br = tf.TransformBroadcaster()

		# Measured Values:
		# All lengths are in inches and angles are in radians
		# self.theta_K_shift = 16.6*(pi/180)	# offset angle, between hip-knee (HK) line and hip-femur ball nut (HN) line
		# self.theta_HKP_shift = 1*(pi/180)	# offset angle, between knee-foot (KP) line and knee-tibia link connection (KL) line
		# self.theta_H = 10.3*(pi/180)		# offset angle, between x-axis and hip constraint (HL)
		# self.theta_t_shift = 15.8*(pi/180)	# offset angle, between knee-tibia ball nut (KN) line and knee-hip (KH) line

		self.theta_K_shift = 13*(pi/180)   #16.6# offset angle, between hip-knee (HK) line and hip-femur ball nut (HN) line
		self.theta_HKP_shift = -1*(pi/180)    #8-1.5#1# offset angle, between knee-foot (KP) line and knee-tibia link connection (KL) line
		self.theta_H = 10.3*(pi/180)         #10.3 # 16.9 # offset angle, between x-axis and hip constraint (HL)
		self.theta_t_shift = 14*(pi/180)   #21.3-7.3 #15.8# offset angle, between knee-tibia ball nut (KN) line and knee-hip (KH) line


		self.length_f = 14.125   # distance from hip to knee pivots
		self.length_t = 13.25    # distance from knee pivots to bottom of foot (P)

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
		self.theta_f = 0  # angle of femur, from femur ball screw to hip constraint
		self.theta_t = 0  # angle of tibia, from tibia ball screw to hip constraint

		# To be calculated...
		self.theta_f_prime = 0
		self.theta_t_prime = 0

		self.theta_HLN = None   # angle between the constraint (HL) and linkage rod (LN)
		self.theta_HNL = None	# angle between femur ball screw (HN) and link (NL)

		self.theta_KLN = None	# angle between knee constraint (KL) and link (LN)
		self.theta_KNL = None	# angle between tibia ball screw (KN) and link (NL)

		# Subscribe to theta_f and theta_t from inverse_kinematics
		# self.sub_F = rospy.Subscriber("/theta_f", Float64Stamped, femur_motor_callback)
		# self.sub_T = rospy.Subscriber("/theta_t", Float64Stamped, tibia_motor_callback)
		self.sub_F = rospy.Subscriber("/theta_f", Float64, self.femur_motor_callback)
		self.sub_T = rospy.Subscriber("/theta_t", Float64, self.tibia_motor_callback)


		# Publish markers for femur and tibia
		self.femur_pub = rospy.Publisher("/femur_marker", Marker, queue_size = 1)
		self.tibia_pub = rospy.Publisher("/tibia_marker", Marker, queue_size = 1)
		self.hip_pub = rospy.Publisher("/hip_marker", Marker, queue_size = 1)
		self.link_connect_pub = rospy.Publisher("/link_connect_marker", Marker, queue_size = 1)
		self.knee_pub = rospy.Publisher("/knee_marker", Marker, queue_size = 1)
		self.foot_pub = rospy.Publisher("/foot_marker", Marker, queue_size = 1)
		self.femur_link_pub = rospy.Publisher("/femur_link_marker", Marker, queue_size = 1)
		self.tibia_link_pub = rospy.Publisher("/tibia_link_marker", Marker, queue_size = 1)
		self.femur_nut_pub = rospy.Publisher("/femur_nut_marker", Marker, queue_size = 1)
		self.tibia_nut_pub = rospy.Publisher("/tibia_nut_marker", Marker, queue_size = 1)


		# Set up a timed loop
		rospy.Timer(rospy.Duration(0.1), self.timer_callback, oneshot=False)

		# Class variables
		self.br = tf.TransformBroadcaster()		



	def femur_motor_callback(self, data):
		self.theta_f = data.data
		self.theta_f_prime = pi - self.theta_f + self.theta_H #TODO: Fill in theta_H (constant offset)

		

	def tibia_motor_callback(self, data):
		self.theta_t = data.data
		# Need to twist back towards world x-axis (so rotate negatively)
		self.theta_t_prime = (pi - self.theta_t - self.theta_t_shift + self.theta_K_shift)  #TODO: Fill in theta_t_shift (constant offset from femur)
		
	def timer_callback(self, data):
 		# The world frame is centered on the hip joint, with positive x in the direction of forward walking...
 		# ...and positive y towards the ground

 		hip = Marker()
 		hip.header.frame_id = "world";
		hip.header.stamp = rospy.Time.now();
		# hip.ns = "my_namespace";
		# hip.id = 0;
		hip.type = 3;
		hip.action = hip.MODIFY;
		hip.pose.position.x = 0;
		hip.pose.position.y = 0;
		hip.pose.position.z = -4;
		hip.pose.orientation.x = 0.0;
		hip.pose.orientation.y = 0.0;
		hip.pose.orientation.z = 0.0;
		hip.pose.orientation.w = 1.0;
		hip.scale.x = 1.25;
		hip.scale.y = 1.25;
		hip.scale.z = 8;
		hip.color.a = 1.0; # Don't forget to set the alpha!
		hip.color.r = 0.0;
		hip.color.g = 1.0;
		hip.color.b = 0.0;
		#only if using a MESH_RESOURCE hip type:
		# hip.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.hip_pub.publish( hip );

		link_connect = Marker()
		link_connect.header.frame_id = "world";
		link_connect.header.stamp = rospy.Time.now();
		# link_connect.ns = "my_namespace";
		# link_connect.id = 0;
		link_connect.type = 3;
		link_connect.action = link_connect.MODIFY;
		link_connect.pose.position.x = -3.875;
		link_connect.pose.position.y = -1.5;
		link_connect.pose.position.z = -4;
		link_connect.pose.orientation.x = 0.0;
		link_connect.pose.orientation.y = 0.0;
		link_connect.pose.orientation.z = 0.0;
		link_connect.pose.orientation.w = 1.0;
		link_connect.scale.x = 0.394;
		link_connect.scale.y = 0.394;
		link_connect.scale.z = 8;
		link_connect.color.a = 1.0; # Don't forget to set the alpha!
		link_connect.color.r = 0.0;
		link_connect.color.g = 1.0;
		link_connect.color.b = 0.0;
		#only if using a MESH_RESOURCE link_connect type:
		# link_connect.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.link_connect_pub.publish( link_connect );

		knee = Marker()
		knee.header.frame_id = "world";
		knee.header.stamp = rospy.Time.now();
		# knee.ns = "my_namespace";
		# knee.id = 0;
		knee.type = 3;
		knee.action = knee.MODIFY;
		knee.pose.position.x = self.length_f*cos(self.theta_f_prime - self.theta_K_shift);
		knee.pose.position.y = self.length_f*sin(self.theta_f_prime - self.theta_K_shift);
		knee.pose.position.z = -4;
		knee.pose.orientation.x = 0.0;
		knee.pose.orientation.y = 0.0;
		knee.pose.orientation.z = 0.0;
		knee.pose.orientation.w = 1.0;
		knee.scale.x = 0.394;
		knee.scale.y = 0.394;
		knee.scale.z = 8;
		knee.color.a = 1.0; # Don't forget to set the alpha!
		knee.color.r = 0.0;
		knee.color.g = 1.0;
		knee.color.b = 0.0;
		#only if using a MESH_RESOURCE knee type:
		# knee.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.knee_pub.publish( knee );

		foot = Marker()
		foot.header.frame_id = "world";
		foot.header.stamp = rospy.Time.now();
		# foot.ns = "my_namespace";
		# foot.id = 0;
		foot.type = 3;
		foot.action = foot.MODIFY;
		foot.pose.position.x = self.length_f*cos(self.theta_f_prime - self.theta_K_shift)+self.length_t*(cos(self.theta_f_prime - self.theta_t_prime));
		foot.pose.position.y = self.length_f*sin(self.theta_f_prime - self.theta_K_shift)+self.length_t*(sin(self.theta_f_prime - self.theta_t_prime));
		foot.pose.position.z = -4;
		foot.pose.orientation.x = 0.0;
		foot.pose.orientation.y = 0.0;
		foot.pose.orientation.z = 0.0;
		foot.pose.orientation.w = 1.0;
		foot.scale.x = 0.5;
		foot.scale.y = 0.5;
		foot.scale.z = 8;
		foot.color.a = 1.0; # Don't forget to set the alpha!
		foot.color.r = 0.0;
		foot.color.g = 1.0;
		foot.color.b = 0.0;
		#only if using a MESH_RESOURCE foot type:
		# foot.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.foot_pub.publish( foot );

 		# Create a new set of coordinates for the femur frame, relative to the world frame.
 		# This too is centered on the hip, but offset in z from the body.
 		self.br.sendTransform((0, 0, -5),
				         tf.transformations.quaternion_from_euler(0, 0, self.theta_f_prime),
				         rospy.Time.now(),
				         'femur',
				         "world")
 		femur = Marker()
		femur.header.frame_id = "femur";
		femur.header.stamp = rospy.Time.now();
		# femur.ns = "my_namespace";
		# femur.id = 0;
		femur.type = femur.CUBE;      # set shape of marker
		femur.action = femur.MODIFY;
		# Indicate the object's position and orientation relative to femur frame's origin
		femur.pose.position.x = 6.5;
		femur.pose.position.y = -2.5;
		femur.pose.position.z = 0;
		femur.pose.orientation.x = 0.0;
		femur.pose.orientation.y = 0.0;
		femur.pose.orientation.z = 0.0;
		femur.pose.orientation.w = 1.0;
		# Indicate size of marker
		femur.scale.x = 16;
		femur.scale.y = 0.875;
		femur.scale.z = 2.5;
		# Set Color (r,g,b)
		femur.color.a = 1.0; # Don't forget to set the alpha!
		femur.color.r = 0.0;
		femur.color.g = 1.0;
		femur.color.b = 0.0;
		#only if using a MESH_RESOURCE femur type:
		# femur.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.femur_pub.publish( femur );

		# Link and Ball Nut Positions
		# Calculating angle between ball screw and link
		self.theta_HNL = arcsin((self.constraint_H * sin(self.theta_f)) / self.link_H)
		# Angle between the hip constraint and linkage rod
		self.theta_HLN = pi - self.theta_f - self.theta_HNL

		# Calculate the distance between the hip and the ball nut
		self.length_HBN = ((self.link_H * sin(self.theta_HLN)) / sin(self.theta_f)) 

		# Check: Make sure the ball nut will not crash into either ball screw mount
		if(self.length_HBN < self.min_H):
			# If less than minimum spacing between ball nut and H, 
			# ball nut will crash into upper ball screw mount...
			# ...so reset value to maximum ball nut position (relative to lower mount.)
			print("Femur ball nut can't go that far! Resetting to maximum position...")
			self.length_HBN = self.min_H
			self.theta_HLN = arcsin((self.length_HBN * sin(self.theta_f)) / self.link_H)
		elif(self.length_HBN > (self.ball_screw_H + self.mount_H)):
			# If less than maximum spacing between ball nut and H, 
			# ball nut will crash into lower ball screw mount...
			# ...so reset value to minimum ball nut position (relative to lower mount).
			print("Femur ball nut can't go that far! Resetting to minimum position...")
			self.length_HBN = self.ball_screw_H + self.mount_H
			self.theta_HLN = arcsin((self.length_HBN * sin(self.theta_f)) / self.link_H)
		else:
			# If in between these two extremes, do nothing.  Everything should be fine!
			pass

		# Create a new set of coordinates for the femur link frame, relative to the world frame.
 		# This is centered on the link connection and offset in z from the body.
 		self.br.sendTransform((-3.875, -1, -6.5),
				         tf.transformations.quaternion_from_euler(0, pi/2, self.theta_HLN + self.theta_H),
				         rospy.Time.now(),
				         'femur_link',
				         "world")
 		femur_link = Marker()
		femur_link.header.frame_id = "femur_link";
		femur_link.header.stamp = rospy.Time.now();
		# femur_link.ns = "my_namespace";
		# femur_link.id = 0;
		femur_link.type = 3;
		femur_link.action = femur_link.MODIFY;
		femur_link.pose.position.x = 0;
		femur_link.pose.position.y = 0;
		femur_link.pose.position.z = 3.4905;
		femur_link.pose.orientation.x = 0.0;
		femur_link.pose.orientation.y = 0.0;
		femur_link.pose.orientation.z = 0.0;
		femur_link.pose.orientation.w = 1.0;
		femur_link.scale.x = 0.394;
		femur_link.scale.y = 0.394;
		femur_link.scale.z = 6.981;
		femur_link.color.a = 1.0; # Don't forget to set the alpha!
		femur_link.color.r = 0.0;
		femur_link.color.g = 1.0;
		femur_link.color.b = 1.0;
		#only if using a MESH_RESOURCE femur_link type:
		# femur_link.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.femur_link_pub.publish( femur_link );


		# Create a new set of coordinates for the femur ball nut frame, relative to the femur frame.
 		# This is centered on the ball nut (where it connects to link).
 		self.br.sendTransform((self.length_HBN, 0, 0),
				         tf.transformations.quaternion_from_euler(0, 0, 0),
				         rospy.Time.now(),
				         'femur_nut',
				         "femur")
 		femur_nut = Marker()
		femur_nut.header.frame_id = "femur_nut";
		femur_nut.header.stamp = rospy.Time.now();
		# femur_nut.ns = "my_namespace";
		# femur_nut.id = 0;
		femur_nut.type = femur_nut.CUBE;
		femur_nut.action = femur_nut.MODIFY;
		femur_nut.pose.position.x = 0;
		femur_nut.pose.position.y = -0.34375;
		femur_nut.pose.position.z = 0;
		femur_nut.pose.orientation.x = 0.0;
		femur_nut.pose.orientation.y = 0.0;
		femur_nut.pose.orientation.z = 0.0;
		femur_nut.pose.orientation.w = 1.0;
		femur_nut.scale.x = 1.75;
		femur_nut.scale.y = 2.5625;
		femur_nut.scale.z = 2.5;
		femur_nut.color.a = 1.0; # Don't forget to set the alpha!
		femur_nut.color.r = 1.0;
		femur_nut.color.g = 1.0;
		femur_nut.color.b = 0.0;
		#only if using a MESH_RESOURCE femur_nut type:
		# femur_nut.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.femur_nut_pub.publish( femur_nut );

		# Tibia
		# Create a new set of coordinates for the tibia frame, relative to the femur frame.
		# This frame is centered on the knee joint.
		self.br.sendTransform((13.5625, -4, 0),
				         tf.transformations.quaternion_from_euler(0, 0, -self.theta_t_prime - self.theta_HKP_shift),
				         rospy.Time.now(),
				         'tibia',
				         "femur")
 		tibia = Marker()
		tibia.header.frame_id = "tibia";
		tibia.header.stamp = rospy.Time.now();
		# tibia.ns = "my_namespace";
		# tibia.id = 0;
		tibia.type = tibia.CUBE;
		tibia.action = tibia.MODIFY;
		tibia.pose.position.x = 6.625;
		tibia.pose.position.y = 1.1875;
		tibia.pose.position.z = 0;
		tibia.pose.orientation.x = 0.0;
		tibia.pose.orientation.y = 0.0;
		tibia.pose.orientation.z = 0.0;
		tibia.pose.orientation.w = 1.0;
		tibia.scale.x = 13.25;
		tibia.scale.y = 0.875;
		tibia.scale.z = 1.5;
		tibia.color.a = 1.0; # Don't forget to set the alpha!
		tibia.color.r = 0.0;
		tibia.color.g = 1.0;
		tibia.color.b = 0.0;
		#only if using a MESH_RESOURCE tibia type:
		# tibia.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.tibia_pub.publish( tibia );

		# Link and Ball Nut Positions
		# Calculate the angle between tibia ball screw (KN) and link (NL)
		self.theta_KNL = arcsin((self.constraint_K * sin(self.theta_t)) / self.link_K)
		# Calculate the angle between knee constraint (KL) and link (LN)
		self.theta_KLN = pi - self.theta_KNL - self.theta_t

		# Calculate the distance between the knee and the ball nut
		self.length_KBN = (self.link_K * sin(self.theta_KLN)) / sin(self.theta_t)

		# Check: Make sure the ball nut will not crash into either ball screw mount
		if(self.length_KBN < self.min_K):
			# If less than minimum spacing between ball nut and K, 
			# ball nut will crash into lower ball screw mount...
			# ...so reset value to minimum ball nut position (relative to lower mount).
			print("Tibia ball nut can't go that far! Resetting to minimum position...")
			self.length_KBN = self.min_K
			self.theta_KLN = arcsin((self.length_KBN * sin(self.theta_t)) / self.link_K)
		elif(self.length_KBN > (self.ball_screw_K + self.mount_K)):
			# If less than maximum spacing between ball nut and K, 
			# ball nut will crash into upper ball screw mount...
			# ...so reset value to maximum ball nut position (relative to lower mount).
			print("Tibia ball nut can't go that far! Resetting to maximum position...")
			self.length_KBN = self.ball_screw_K + self.mount_K
			self.theta_KLN = arcsin((self.length_KBN * sin(self.theta_t)) / self.link_K)
		else:
			# If in between these two extremes, do nothing.  Everything should be fine!
			pass

		# Create a new set of coordinates for the tibia link frame, relative to the tibia frame.
		# This frame is centered on the connection between the link and tibia.
		self.br.sendTransform((3.75, 0.644, -1.5),
				         tf.transformations.quaternion_from_euler(0, pi/2, -pi + self.theta_KLN - self.theta_HKP_shift),
				         rospy.Time.now(),
				         'tibia_link',
				         "tibia")
 		tibia_link = Marker()
		tibia_link.header.frame_id = "tibia_link";
		tibia_link.header.stamp = rospy.Time.now();
		# tibia_link.ns = "my_namespace";
		# tibia_link.id = 0;
		tibia_link.type = 3;
		tibia_link.action = tibia_link.MODIFY;
		tibia_link.pose.position.x = 0;
		tibia_link.pose.position.y = 0;
		tibia_link.pose.position.z = 3.4905;
		tibia_link.pose.orientation.x = 0.0;
		tibia_link.pose.orientation.y = 0.0;
		tibia_link.pose.orientation.z = 0.0;
		tibia_link.pose.orientation.w = 1.0;
		tibia_link.scale.x = 0.394;
		tibia_link.scale.y = 0.394;
		tibia_link.scale.z = 6.981;
		tibia_link.color.a = 1.0; # Don't forget to set the alpha!
		tibia_link.color.r = 0.0;
		tibia_link.color.g = 1.0;
		tibia_link.color.b = 1.0;
		#only if using a MESH_RESOURCE tibia_link type:
		# tibia_link.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.tibia_link_pub.publish( tibia_link );

		# Create a new set of coordinates for the tibia ball nut frame, relative to the femur frame.
 		# This is centered on the ball nut (where it connects to link).
 		self.br.sendTransform((13.5625- self.length_KBN, -4, 0),
				         tf.transformations.quaternion_from_euler(0, 0, 0),
				         rospy.Time.now(),
				         'tibia_nut',
				         "femur")
 		tibia_nut = Marker()
		tibia_nut.header.frame_id = "tibia_nut";
		tibia_nut.header.stamp = rospy.Time.now();
		# tibia_nut.ns = "my_namespace";
		# tibia_nut.id = 0;
		tibia_nut.type = tibia_nut.CUBE;
		tibia_nut.action = tibia_nut.MODIFY;
		tibia_nut.pose.position.x = 0;
		tibia_nut.pose.position.y = -0.34375;
		tibia_nut.pose.position.z = 0;
		tibia_nut.pose.orientation.x = 0.0;
		tibia_nut.pose.orientation.y = 0.0;
		tibia_nut.pose.orientation.z = 0.0;
		tibia_nut.pose.orientation.w = 1.0;
		tibia_nut.scale.x = 1.75;
		tibia_nut.scale.y = 2.5625;
		tibia_nut.scale.z = 2.5;
		tibia_nut.color.a = 1.0; # Don't forget to set the alpha!
		tibia_nut.color.r = 1.0;
		tibia_nut.color.g = 1.0;
		tibia_nut.color.b = 0.0;
		#only if using a MESH_RESOURCE tibia_nut type:
		# tibia_nut.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		self.tibia_nut_pub.publish( tibia_nut );

def main(args):
	rospy.init_node('motor_position',anonymous=True)
	MP = MotorPosition()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
