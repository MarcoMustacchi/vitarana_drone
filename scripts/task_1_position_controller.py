#!/usr/bin/env python

import rospy
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from pid_tune.msg import PidTune
from std_msgs.msg import Float32

class Edrone ():

	def __init__(self,):

		self.actual_location = [0.0, 0.0, 0.0]
		self.desired_location = [19.0000451704, 72.0, 3.0]

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		self.error = [0.0, 0.0, 0.0]
		self.error_sum = [0.0, 0.0, 0.0]
		self.error_change = [0.0, 0.0, 0.0]

		self.rpyt_cmd = edrone_cmd()
		self.rpyt_cmd.rcRoll = 0.0
		self.rpyt_cmd.rcPitch = 0.0
		self.rpyt_cmd.rcYaw = 0.0
		self.rpyt_cmd.rcThrottle = 0.0

		self.zero_error = Float32()
		self.roll_error = Float32()
		self.pitch_error = Float32()
		self.yaw_error = Float32()
		self.zero_error = 0.0
		self.x_error = 0.0
		self.y_error = 0.0
		self.z_error = 0.0
		
		# Subscribers
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/rpid_params', PidTune, self.set_pid_value_roll)
        	rospy.Subscriber('/ppid_params', PidTune, self.set_pid_value_pitch)
        	rospy.Subscriber('/ypid_params', PidTune, self.set_pid_value_yaw)

		# Publishers
		self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
		self.zero_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
		self.x_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
		self.y_error_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
		self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)

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

