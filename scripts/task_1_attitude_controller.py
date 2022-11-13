#!/usr/bin/env python

import rospy
import tf
from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from pid_tune.msg import PidTune
from std_msgs.msg import Float32

class Edrone ():

	def __init__(self,):

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


		#____________________Subscribers____________________
		rospy.Subscriber('/edrone/imu', Imu, self.imu_callback)
		rospy.Subscriber('/rpid_params', PidTune, self.set_pid_value_roll)
        	rospy.Subscriber('/ppid_params', PidTune, self.set_pid_value_pitch)
        	rospy.Subscriber('/ypid_params', PidTune, self.set_pid_value_yaw)

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
		(actual_euler_orientation[0], actual_euler_orientation[1], actual_euler_orientation[2]) = tf.transformation.euler_from_quaternion(self.actual_quaternion_orientation[0], self.actual_quaternion_orientation[1], self.actual_quaternion_orientation[2], self.actual_quaternion_orientation[3])


	# Callback for getting outputs of Position Controller
	def rpyt_cmd_callback(self, msg):
		self.setpoint_rpyt_cmd[0] = msg.rcRoll
		self.setpoint_rpyt_cmd[1] = msg.rcPitch
		self.setpoint_rpyt_cmd[2] = msg.rcYaw
		self.throttle_pos_cmd = msg.rcThrottle

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

	# ____________________Methods____________________
	# Updating errors for PID
	def error_update(self):
	        for i in range(3):
		    self.error[i] = self.desired_euler_orientation[i] - self.actual_euler_orientation[i]
		    self.error_value[i] = self.error_sum[i] + self.error[i]
		    self.error_change[i] = self.error[i] - self.previous_error[i]
		    self.previous_error[i] = self.error[i]

		self.roll_error.data = self.error[0]
		self.pitch_error.data = self.error[1]
		self.yaw_error.data = self.error[2]

		self.zero_error_pub.publish(zero_error)
		self.roll_error_pub.publish(roll_error)
		self.pitch_error_pub.publish(pitch_error)
		self.yaw_error_pub.publish(yaw_error)
		
	# Controller PID 
	def controller(self):
		latitude_cmd = self.Kp[0]*self.error + self.Ki[0]*self.error_sum + self.Kd[0]*self.error_change
		longitude_cmd = self.Kp[1]*self.error + self.Ki[1]*self.error_sum + self.Kd[1]*self.error_change
		altitude_cmd = self.Kp[2]*self.error + self.Ki[2]*self.error_sum + self.Kd[2]*self.error_change

	# Output commands
	def out_commands(self):
		self.pwm_cmd.prop1 = throttle_pos_cmd - latitude_cmd + longitude_cmd - altitude_cmd
		self.pwm_cmd.prop2 = throttle_pos_cmd - latitude_cmd - longitude_cmd + altitude_cmd
		self.pwm_cmd.prop3 = throttle_pos_cmd + latitude_cmd - longitude_cmd - altitude_cmd
		self.pwm_cmd.prop4 = throttle_pos_cmd + latitude_cmd + longitude_cmd + altitude_cmd		

		self.pwm_cmd_pub.publish(self.pwm_cmd)
		


if __name__ == '__main__':

	drone = Edrone()

