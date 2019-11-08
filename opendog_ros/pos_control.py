#!/usr/bin/env python

#basics
import rospy
import sys
import roslib
roslib.load_manifest('opendog_ros')

#load all the message types you need to publish or subscribe to
from std_msgs.msg import String
from std_msgs.msg import Float32  # std_msgs don't have stamped? http://answers.ros.org/question/9715/stamped-std_msgs/
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

#import any relevant python packages for you (numpy, opencv, etc.)
import numpy as np



class pos_control:
	def __init__(self):

		#if you need parameters, use the following
		#self.mything = rospy.get_param('param_name',default_value)

		#now set up any subscribers
		# desired position (degrees) from tF_des topic (theta femur) published by ____(node TBD)__
		self.sub_des = rospy.Subscriber("tF_des",Float32,self.des_callback)
		# actual position (counts) from /encoder_right topic (encoder position) published by odrive_node.py
		self.sub_act = rospy.Subscriber("odrive/raw_odom/encoder_right", Int32, self.act_callback)
		
		# Define class-owned variables
		# Position variables
		self.des_pos = None
		self.act_pos = 0.
		self.rawnow = None
		self.rawlast = None
		self.init_pos = None
		
		self.k = 5.  # damping
		self.laps = 0  # tracks number of revolutions
		self.maxspeed = 19000  # velocity limit (counts/s)

		# Conversions
		self.deg_to_rad = np.pi / 180
		self.rad_to_deg = 180 / np.pi
		self.count_to_rad = (2 * np.pi) / 8192
		self.rad_to_count = 8192 / (2 * np.pi)

		
		#now set up your publisher
		#this publisher will publish on a timer.
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		#now set up a timed loop
		rospy.Timer(rospy.Duration(0.01), self.timercallback, oneshot=False)


	def des_callback(self,data):
		#right now, this time is LOCAL. I can't access it from another function in the class.
		#time_this_happened = data.header.stamp
		
		# Assign subscribed desired position value to class variable, convert to radians
		self.des_pos = data.data * self.deg_to_rad * self.rad_to_count
		print("Des: %s", self.des_pos)

	def act_callback(self,data):
		# Assign subscribed actual position value to class variable, convert to radians
		if(self.init_pos is None):
			# If init_pos has not yet been assigned a value...
			# ...set it to the current position.
			self.rawnow = data.data
			self.rawlast = data.data
			self.init_pos = data.data
			self.act_pos = self.rawnow - self.init_pos   # Should be 0 initially
			# print("Act: %s", self.act_pos)   # Debugging
		else:
			# If init_pos has a value...
			# ...calculate how many laps the motor has completed.
			self.rawnow = data.data
			if(abs(self.rawnow - self.rawlast) > 6):
				self.laps-=np.sign(self.rawnow - self.rawlast)

			# Then calculate the actual position.
			self.act_pos = (self.rawnow - self.init_pos) + (2 * np.pi * self.laps)
			self.rawlast = self.rawnow         # reset rawlast to current value
			# print("Act: %s", self.act_pos)   # Debugging

	def timercallback(self,data):
		# Calculate velocity, u.

		if(self.des_pos is not None):
			# If the node has received a value for desired position...
			# ...calculate u.
			u = self.k * (self.des_pos-self.act_pos)
			
			# Check if u has surpassed the maximum speed
			if(abs(u)>self.maxspeed):
				u = np.sign(u) * self.maxspeed
				print("Limited!")

			print(u)  # Debugging
			
			# Create ouput message type and publish it
			output = Twist()
			# output.header.stamp = rospy.Time.now()
			output.angular.z = u
			self.pub.publish(output)
		else:
			pass

def main(args):
	rospy.init_node('pos_control',anonymous=True)
	PS = pos_control()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)


