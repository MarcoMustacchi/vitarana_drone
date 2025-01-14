#!/usr/bin/env python

import rospy
import tf
import time
import math
from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from pid_tune.msg import PidTune
from std_msgs.msg import Float32

class Edrone ():

	def __init__(self,):

		#____________________Node____________________
		rospy.init_node('attitude_controller')

		#____________________Variables____________________ 
		self.actual_quaternion_orientation = [0.0, 0.0, 0.0, 0.0]
		self.actual_euler_orientation = [0.0, 0.0, 0.0]
		self.desired_euler_orientation = [0.0, 0.0, 0.0]

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
		self.roll_error = Float32()
		self.pitch_error = Float32()
		self.yaw_error = Float32()
		self.zero_error.data = 0.0
		self.roll_error.data = 0.0
		self.pitch_error.data = 0.0
		self.yaw_error.data = 0.0

		# Output commands
		self.pwm_cmd = prop_speed()
		self.pwm_cmd.prop1 = 0.0
		self.pwm_cmd.prop2 = 0.0
		self.pwm_cmd.prop3 = 0.0
		self.pwm_cmd.prop4 = 0.0
		
		# Input commands from Position Controller
		self.setpoint_rpyt_cmd = [0.0, 0.0, 0.0]
		self.throttle_pos_cmd = 0.0

		# Desired Orientation Tolerance
		self.ori_tolerance = [0.0, 0.0, 0.0]

		# Controller Sample Time
		self.sample_time = 0.05  # in seconds


		#____________________Subscribers____________________
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
		rospy.Subscriber('/drone_command', edrone_cmd, self.rpyt_cmd_callback)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_value_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_value_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_value_yaw)

		# ____________________Publishers____________________
		self.pwm_cmd_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
		self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
		self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)

	# ____________________Callbacks____________________
	def imu_callback(self, msg):
		self.actual_quaternion_orientation[0] = msg.orientation.x
		self.actual_quaternion_orientation[1] = msg.orientation.y
		self.actual_quaternion_orientation[2] = msg.orientation.z
		self.actual_quaternion_orientation[3] = msg.orientation.w

		(self.actual_euler_orientation[1], self.actual_euler_orientation[0], self.actual_euler_orientation[2]) = tf.transformations.euler_from_quaternion([self.actual_quaternion_orientation[0], self.actual_quaternion_orientation[1], self.actual_quaternion_orientation[2], self.actual_quaternion_orientation[3]])

		# Converting radians to degrees
		# self.actual_euler_orientation[0]=math.degrees(self.actual_euler_orientation[0])
		# self.actual_euler_orientation[1]=math.degrees(self.actual_euler_orientation[1])
		# self.actual_euler_orientation[2]=math.degrees(self.actual_euler_orientation[2])


	# Callback for getting outputs of Position Controller
	def rpyt_cmd_callback(self, msg):
		self.setpoint_rpyt_cmd[0] = msg.rcRoll
		self.setpoint_rpyt_cmd[1] = msg.rcPitch
		self.setpoint_rpyt_cmd[2] = msg.rcYaw
		self.throttle_pos_cmd = msg.rcThrottle

		# Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll, pitch, yaw axis
		self.desired_euler_orientation[0] = self.setpoint_rpyt_cmd[0] * 0.02 - 30
		self.desired_euler_orientation[1] = self.setpoint_rpyt_cmd[1] * 0.02 - 30
		self.desired_euler_orientation[2] = self.setpoint_rpyt_cmd[2] * 0.02 - 30

	# Callback functions for /pid_tuning
	def set_pid_value_roll(self, data):
		self.Kp[0] = data.Kp 
		self.Kd[0] = data.Kd
		self.Ki[0] = data.Ki

	def set_pid_value_pitch(self, data):
		self.Kp[1] = data.Kp 
		self.Kd[1] = data.Kd 
		self.Ki[1] = data.Ki

	def set_pid_value_yaw(self, data):
		self.Kp[2] = data.Kp 
		self.Kd[2] = data.Kd 
		self.Ki[2] = data.Ki

	# ____________________Methods____________________		
	# Controller PID 
	def pid(self):
		# Updating errors for PID and Publish
		for i in range(3):
		    self.error[i] = self.desired_euler_orientation[i] - self.actual_euler_orientation[i]
		    self.error_sum[i] = self.error_sum[i] + self.error[i]
		    self.error_change[i] = self.error[i] - self.previous_error[i]
		    self.previous_error[i] = self.error[i]

		self.roll_error.data = self.error[0]
		self.pitch_error.data = self.error[1]
		self.yaw_error.data = self.error[2]

		self.zero_error_pub.publish(self.zero_error)
		self.roll_error_pub.publish(self.roll_error)
		self.pitch_error_pub.publish(self.pitch_error)
		self.yaw_error_pub.publish(self.yaw_error)

		# PID Control
		latitude_cmd = self.Kp[0]*self.error[0] + self.Ki[0]*self.error_sum[0] + self.Kd[0]*self.error_change[0]
		longitude_cmd = self.Kp[1]*self.error[1] + self.Ki[1]*self.error_sum[1] + self.Kd[1]*self.error_change[1]
		altitude_cmd = self.Kp[2]*self.error[2] + self.Ki[2]*self.error_sum[2] + self.Kd[2]*self.error_change[2]

		# Conversion from 1000-2000 to 0-1024
		throttle_att_cmd = (self.throttle_pos_cmd*1.024)  - 1024.0

		# Output commands
		prop1 = throttle_att_cmd - latitude_cmd + longitude_cmd - altitude_cmd
		prop2 = throttle_att_cmd - latitude_cmd - longitude_cmd + altitude_cmd
		prop3 = throttle_att_cmd + latitude_cmd - longitude_cmd - altitude_cmd
		prop4 = throttle_att_cmd + latitude_cmd + longitude_cmd + altitude_cmd

		# Handle output commands Saturation and Publish 
		if(prop1 > 1024):
		    self.pwm_cmd.prop1 = 1024
		elif(prop1 < 0):
		    self.pwm_cmd.prop1 = 0
		else:
		    self.pwm_cmd.prop1 = prop1

		if(prop2 > 1024):
		    self.pwm_cmd.prop2 = 1024
		elif(prop2 < 0):
		    self.pwm_cmd.prop2 = 0
		else:
		    self.pwm_cmd.prop2 = prop2

		if(prop3 > 1024):
		    self.pwm_cmd.prop3 = 1024
		elif(prop3 < 0):
		    self.pwm_cmd.prop3 = 0
		else:
		    self.pwm_cmd.prop3 = prop3

		if(prop4 > 1024):
		    self.pwm_cmd.prop4 = 1024
		elif(prop4 < 0):
		    self.pwm_cmd.prop4 = 0
		else:
		    self.pwm_cmd.prop4 = prop4


		self.pwm_cmd_pub.publish(self.pwm_cmd)

# ____________________Main____________________
if __name__ == '__main__':

	drone = Edrone()

	rospy.loginfo("Drone started from orientation: " + str(drone.actual_euler_orientation))

	while not rospy.is_shutdown():
		drone.pid()
		time.sleep(drone.sample_time)

