#!/usr/bin/env python

import rospy
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix


class Edrone ():

	def __init__(self,):
		self.actual_location = [0.0, 0.0, 0.0]
		self.desired_location = [19.0000451704, 72.0, 3.0]

	rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)

	self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

	def gps_callback(self):
		self.actual_location[0] = msg.latitude
		self.actual_location[1] = msg.longitude
		self.actual_location[2] = msg.altitude


if __name__ == '__main__':

	drone = Edrone()

