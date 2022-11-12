#!/usr/bin/env python

import rospy
import tf
from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from pid_tune.msg import PidTune

class Edrone ():

	def __init__(self,):

		self.actual_quaternion_orientation = [0.0, 0.0, 0.0, 0.0]
		self.actual_euler_orientation = [0.0, 0.0, 0.0]
		self.desired_euler_orientation = [0.0, 0.0, 0.0]

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		rospy.Subscriber('/edrone/imu', Imu, self.imu_callback)

		self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

	def imu_callback(self, msg):
		self.actual_quaternion_orientation[0] = msg.orientation.x
		self.actual_quaternion_orientation[1] = msg.orientation.y
		self.actual_quaternion_orientation[2] = msg.orientation.z
		self.actual_quaternion_orientation[3] = msg.orientation.w
		(actual_euler_orientation[0], actual_euler_orientation[1], actual_euler_orientation[2]) = tf.transformation.euler_from_quaternion(self.actual_quaternion_orientation[0], self.actual_quaternion_orientation[1], self.actual_quaternion_orientation[2], self.actual_quaternion_orientation[3])

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

