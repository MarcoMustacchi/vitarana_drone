#!/usr/bin/env python

import rospy
import tf
import numpy as np
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from pid_tune.msg import PidTune
from std_msgs.msg import Float32

class Edrone ():

	def __init__(self,):

		#____________________Node____________________
		rospy.init_node('position_controller')

		#____________________Variables____________________ 
		self.actual_location = [0.0, 0.0, 0.0]
		self.desired_location = [19.0, 72.0, 3.0]

		self.actual_quaternion_orientation = [0.0, 0.0, 0.0, 0.0]
		self.actual_euler_orientation = [0.0, 0.0, 0.0]

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		# Errors for PID
		self.error = [0.0, 0.0, 0.0]
		self.error_sum = [0.0, 0.0, 0.0]
		self.error_change = [0.0, 0.0, 0.0]
		self.previous_error = [0.0, 0.0, 0.0]

		# Errors to publish for PID tuning
		self.zero_error = Float32()
		self.x_error = Float32()
		self.y_error = Float32()
		self.z_error = Float32()
		self.zero_error.data = 0.0
		self.x_error.data = 0.0
		self.y_error.data = 0.0
		self.z_error.data = 0.0

		# Output commands
		self.rpyt_cmd = edrone_cmd()
		self.rpyt_cmd.rcRoll = 0.0
		self.rpyt_cmd.rcPitch = 0.0
		self.rpyt_cmd.rcYaw = 0.0
		self.rpyt_cmd.rcThrottle = 0.0


		#____________________Subscribers____________________
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_value_roll)
        	rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_value_pitch)
        	rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_value_yaw)
		# rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_value_throttle)

		# ____________________Publishers____________________
		self.rpyt_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
		self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
		self.x_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
		self.y_error_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
		self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
	
	# ____________________Callbacks____________________
	def gps_callback(self, msg):
		self.actual_location[0] = msg.latitude
		self.actual_location[1] = msg.longitude
		self.actual_location[2] = msg.altitude

	def imu_callback(self, msg): # same as attitude controller
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
		rospy.loginfo("drone PID pitch changed to Kp: " + str(self.Kp[1]) + "Ki: " + str(self.Ki[1]) + "Kd: " + str(self.Kd[1]))
		self.Kp[1] = data.Kp 
		self.Kd[1] = data.Kd 
		self.Ki[1] = data.Ki

	def set_pid_value_yaw(self, data):
		rospy.loginfo("drone PID yaw changed to Kp: " + str(self.Kp[2]) + "Ki: " + str(self.Ki[2]) + "Kd: " + str(self.Kd[2]))
		self.Kp[2] = data.Kp 
		self.Kd[2] = data.Kd 
		self.Ki[2] = data.Ki

	# ____________________Methods____________________
	# Controller PID 
	def pid(self):
		# Updating errors for PID and Publish
		for i in range(3):
		    self.error[i] = self.desired_location[i] - self.actual_location[i]
		    self.error_sum[i] = self.error_sum[i] + self.error[i]
		    self.error_change[i] = self.error[i] - self.previous_error[i]
		    self.previous_error[i] = self.error[i]
		
		self.x_error.data = self.error[0]
		self.y_error.data = self.error[1]
		self.z_error.data = self.error[2]

		self.zero_error_pub.publish(self.zero_error)
		self.x_error_pub.publish(self.x_error)
		self.y_error_pub.publish(self.y_error)
		self.z_error_pub.publish(self.z_error)

		# PID Control
		latitude_cmd = self.Kp[0]*self.error[0] + self.Ki[0]*self.error_sum[0] + self.Kd[0]*self.error_change[0]
		longitude_cmd = self.Kp[1]*self.error[1] + self.Ki[1]*self.error_sum[1] + self.Kd[1]*self.error_change[1]
		altitude_cmd = self.Kp[2]*self.error[2] + self.Ki[2]*self.error_sum[2] + self.Kd[2]*self.error_change[2]

		# Output commands
		self.rpyt_cmd.rcRoll = 1500 + latitude_cmd*np.cos(self.actual_euler_orientation[2]) - longitude_cmd*np.sin(self.actual_euler_orientation[2])
		self.rpyt_cmd.rcPitch = 1500 + latitude_cmd*np.sin(self.actual_euler_orientation[2]) + longitude_cmd*np.cos(self.actual_euler_orientation[2])
		self.rpyt_cmd.rcThrottle = 1500 + altitude_cmd

		# Handle output commands Saturation and Publish
		if(self.rpyt_cmd.rcRoll > 1800):
		    self.rpyt_cmd.rcRoll = 1800
		elif(self.rpyt_cmd.rcRoll < 1200):
		    self.rpyt_cmd.rcRoll = 1200

		if(self.rpyt_cmd.rcPitch > 1800):
		    self.rpyt_cmd.rcPitch = 1800
		elif(self.rpyt_cmd.rcPitch < 1200):
		    self.rpyt_cmd.rcPitch = 1200

		if(self.rpyt_cmd.rcThrottle > 2000):
		    self.rpyt_cmd.rcThrottle = 2000
		elif(self.rpyt_cmd.rcThrottle < 1000):
		    self.rpyt_cmd.rcThrottle = 1000

		self.rpyt_cmd_pub.publish(self.rpyt_cmd)


# ____________________Main____________________
def main():
	drone.pid()

if __name__ == '__main__':

	drone = Edrone()

	while not rospy.is_shutdown():
		main()

