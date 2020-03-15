#!/usr/bin/env python

from numpy import *
import time
import Queue   # might not be needed

#basics
import rospy
import sys
import roslib
roslib.load_manifest('odrive_ros')

# Imports message type
from geometry_msgs.msg import PoseStamped


class FootPath:
	def __init__(self):

		self.foot_position = rospy.get_param('~foot_position', "/footPosition_1")
		self.delay =rospy.get_param('~delay', "/0")

		self.phase_shift = 0

		#create the sparse path. 
		self.S = array([0,3,9,12,18])
		# self.X = array([2,2,8,8,2])
		self.X = array([3,3,9,9,3])
		self.Y = array([25,22,22,25,25])

		#linear speed (constant)
		self.U = 6	#inches per second

		# Initialize "current" values
		self.Snow,self.xnow,self.ynow,self.tnow = 0,0,0,0

		# Initialize number of cycles (laps)
		self.Laps = 0

		# Set up a publisher
		self.pub = rospy.Publisher(self.foot_position, PoseStamped, queue_size = 1)

		# Set up a timed loop
		rospy.Timer(rospy.Duration(0.02), self.timer_callback, oneshot=False)



	def timer_callback(self, data):
		try:
			time.sleep(0)
			#what is the time now?
			# self.tnow = rospy.Time.now()
			self.tnow = time.time()

			Smax = self.S[-1]#very last point in the S vector is the max
			#what is the TOTAL distance traveled?
			self.Snow = self.tnow * self.U
			#how many laps have we done?
			self.Laps = floor(self.Snow/Smax)
			#where are we at on THIS lap?
			Srelative = self.Snow - (self.Laps*Smax)
			#now where should the foot be?
			self.xnow = interp(Srelative,self.S,self.X) - self.phase_shift
			self.ynow = interp(Srelative,self.S,self.Y) - self.phase_shift

			# Create and publish PoseStamped message containing the (x,y) position of the foot
			# Eventually will include z when hip motion is included
			footPosition = PoseStamped()
			# footPosition.header.stamp = rospy.Time.now()
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


def main(args):
	rospy.init_node('pub_foot_path',anonymous=True)
	FP = FootPath()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)