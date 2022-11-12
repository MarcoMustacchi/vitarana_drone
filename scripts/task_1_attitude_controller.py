#!/usr/bin/env python

import rospy
import tf
from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from pid_tune.msg import PidTune
from std_msgs.msg import Float32

class Edrone ():

	def __init__(self,):

		self.actual_quaternion_orientation = [0.0, 0.0, 0.0, 0.0]
		self.actual_euler_orientation = [0.0, 0.0, 0.0]
		self.desired_euler_orientation = [0.0, 0.0, 0.0]

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		self.error = [0.0, 0.0, 0.0]
		self.error_sum = [0.0, 0.0, 0.0]
		self.error_change = [0.0, 0.0, 0.0]

		self.pwm_cmd = prop_speed()
		self.pwm_cmd.prop1 = 0.0
		self.pwm_cmd.prop2 = 0.0
		self.pwm_cmd.prop3 = 0.0
		self.pwm_cmd.prop4 = 0.0


		self.zero_error = Float32()
		self.roll_error = Float32()
		self.pitch_error = Float32()
		self.yaw_error = Float32()
		self.zero_error.data = 0.0
		self.roll_error.data = 0.0
		self.pitch_error.data = 0.0
		self.yaw_error.data = 0.0

		# Subscribers
		rospy.Subscriber('/edrone/imu', Imu, self.imu_callback)
		rospy.Subscriber('/rpid_params', PidTune, self.set_pid_value_roll)
        	rospy.Subscriber('/ppid_params', PidTune, self.set_pid_value_pitch)
        	rospy.Subscriber('/ypid_params', PidTune, self.set_pid_value_yaw)

		# Publishers
		self.pwm_cmdpub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
		self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
		self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)

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

