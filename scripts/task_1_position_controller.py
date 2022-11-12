#!/usr/bin/env python

import rospy
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from pid_tune.msg import PidTune


class Edrone ():

	def __init__(self,):

		self.actual_location = [0.0, 0.0, 0.0]
		self.desired_location = [19.0000451704, 72.0, 3.0]

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)

		self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

	def gps_callback(self):
		self.actual_location[0] = msg.latitude
		self.actual_location[1] = msg.longitude
		self.actual_location[2] = msg.altitude

	# Callback functions for /pid_tuning

	def set_pid_value_roll(self, data):
	rospy.loginfo("drone PID roll changed to Kp: " + str(self.Kp[0]) + "Ki: " + str(self.Ki[0]) + "Kd: " + str(self.Kd[0]))
	self.Kp[0] = data.Kp 
	self.Kd[0] = data.Kd
	self.Ki[0] = data.Ki

	def set_pid_value_pitch(self, data):
	rospy.loginfo("drone PID roll changed to Kp: " + str(self.Kp[1]) + "Ki: " + str(self.Ki[1]) + "Kd: " + str(self.Kd[1]))
	self.Kp[1] = data.Kp 
	self.Kd[1] = data.Kd 
	self.Ki[1] = data.Ki

	def set_pid_value_yaw(self, data):
	rospy.loginfo("drone PID roll changed to Kp: " + str(self.Kp[2]) + "Ki: " + str(self.Ki[2]) + "Kd: " + str(self.Kd[2]))
	self.Kp[2] = data.Kp 
	self.Kd[2] = data.Kd 
	self.Ki[2] = data.Ki


if __name__ == '__main__':

	drone = Edrone()

