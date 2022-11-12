#!/usr/bin/env python

import rospy
import tf
from vitarana_drone.msg import *
from sensor_msgs.msg import Imu

class Edrone ():

	def __init__(self,):
		self.actual_quaternion_orientation = [0.0, 0.0, 0.0, 0.0]
		self.actual_euler_orientation = [0.0, 0.0, 0.0]
		self.desired_euler_orientation = [0.0, 0.0, 0.0]

	rospy.Subscriber('/edrone/imu', Imu, self.imu_callback)

	self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

	def imu_callback(self, msg):
		self.actual_quaternion_orientation[0] = msg.orientation.x
		self.actual_quaternion_orientation[1] = msg.orientation.y
		self.actual_quaternion_orientation[2] = msg.orientation.z
		self.actual_quaternion_orientation[3] = msg.orientation.w
		(actual_euler_orientation[0], actual_euler_orientation[1], actual_euler_orientation[2]) = tf.transformation.euler_from_quaternion(self.actual_quaternion_orientation[0], self.actual_quaternion_orientation[1], self.actual_quaternion_orientation[2], self.actual_quaternion_orientation[3])


if __name__ == '__main__':

	drone = Edrone()

