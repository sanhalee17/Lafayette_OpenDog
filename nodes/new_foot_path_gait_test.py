#!/usr/bin/env python

from numpy import *
import time
import Queue   # might not be needed

#basics
import rospy
import sys
import roslib
import math
roslib.load_manifest('odrive_ros')


# Imports message type
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class FootPath:
	def __init__(self):

		self.foot_position = rospy.get_param('~foot_position', "/footPosition_1")
		self.phase_shift = rospy.get_param('~phase_shift', 0.0)

		# define parameters for sinusoidal path

		self.leg_pace = 2.0 # pace of gait

		self.gait_sub = rospy.Subscriber('/gait_params', Array, self.gait_callback)

		self.x_center = -0.5
		self.x_stride = -4.0

		self.y_center = 25.0
		self.y_lift = 3.0


		# Initialize "current" values
		self.ynow,self.tnow = 0.0, 0.0


		# Set up a publisher
		self.pub = rospy.Publisher(self.foot_position, PoseStamped, queue_size = 1)

		# Set up a timed loop
		rospy.Timer(rospy.Duration(0.02), self.timer_callback, oneshot=False)


	def timer_callback(self, data):
		try:
			#what is the time now?
			# self.tnow = rospy.Time.now()
			self.tnow = time.time()

			# now where should the foot be?
			# rospy.logwarn(self.x_center, self.x_stride, self.leg_pace, self.tnow, self.phase_shift)
			self.xnow = -(self.x_center + self.x_stride*sin(self.leg_pace*self.tnow - self.phase_shift))
			#rospy.logwarn(str(self.xnow) + ' , ' + str(self.ynow))


			self.ynow = (self.y_center + self.y_lift*sin(self.leg_pace*self.tnow - self.phase_shift - pi/2))

			if (self.ynow) > self.y_center: self.ynow = self.y_center

			# Create and publish PoseStamped message containing the (x,y) position of the foot
			# Eventually will include z when hip motion is included
			footPosition = PoseStamped()
			footPosition.header.stamp = rospy.Time.now()
			footPosition.pose.position.x = self.xnow
			footPosition.pose.position.y = self.ynow
			footPosition.pose.position.z = 0.0
			footPosition.pose.orientation.x = 0.0
			footPosition.pose.orientation.y = 0.0
			footPosition.pose.orientation.z = 0.0
			footPosition.pose.orientation.w = 0.0
			self.pub.publish(footPosition)

			# time.sleep(.01)

		except KeyboardInterrupt:
			#close('all')
			#break
			pass

	def gait_callback(self,data):

		self.x_center = data.data[0]
		self.x_stride = data.data[1]
		self.y_center = data.data[2]
		self.y_lift = data.data[3]



def main(args):
	rospy.init_node('pub_foot_path',anonymous=True)
	FP = FootPath()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)
