#!/usr/bin/env python

import time
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from robotx_gazebo.msg import UsvDrive
x_pos = 0
y_pos = 0
x_vel = 0
y_vel = 0
ang_left = 0
ang_right = 0
class PID:

	def __init__(self, pos_P=0.7, pos_I=0.0, pos_D=0.2, vel_P=0.2, vel_I=0.0, vel_D=0.2):

		self.is_close_distance = 3
		self.pos_Kp = pos_P
		self.pos_Ki = pos_I
		self.pos_Kd = pos_D
		
		self.vel_Kp = vel_P
		self.vel_Ki = vel_I
		self.vel_Kd = vel_D

		self.sample_time = 0.00
		self.pos_current_time = time.time()
		self.pos_last_time = self.pos_current_time
		
		self.vel_current_time = time.time()
		self.vel_last_time = self.vel_current_time

		self.clear()

	def clear(self):
		self.pos_SetPointx = 0.0
		self.pos_SetPointy = 0.0
		self.pos_PTerm = 0.0
		self.pos_ITerm = 0.0
		self.pos_DTerm = 0.0
		self.pos_last_error = 0.0

		self.vel_PTerm = 0.0
		self.vel_ITerm = 0.0
		self.vel_DTerm = 0.0
		self.vel_last_error = 0.0
		
		# Windup Guard
		self.int_error = 0.0
		self.windup_guard = 20.0

		self.output = 0.0

	def update(self, x_pos, y_pos, x_vel, y_vel):

		pos_error = np.sqrt((self.pos_SetPointx - x_pos)*(self.pos_SetPointx - x_pos)+(self.pos_SetPointy - y_pos)*(self.pos_SetPointy - y_pos))
		if np.abs(pos_error) < 1:
			pos_error = 0
		print pos_error
		self.pos_current_time = time.time()
		pos_delta_time = self.pos_current_time - self.pos_last_time
		pos_delta_error = pos_error - self.pos_last_error

		if (pos_delta_time >= self.sample_time):
			self.pos_PTerm = self.pos_Kp * pos_error
			self.pos_ITerm += pos_error * pos_delta_time

			if (self.pos_ITerm < -self.windup_guard):
				self.pos_ITerm = -self.windup_guard
			elif (self.pos_ITerm > self.windup_guard):
				self.pos_ITerm = self.windup_guard

			self.DTerm = 0.0
			if pos_delta_time > 0:
				if pos_delta_error > self.is_close_distance:
					self.pos_DTerm = 500
				else:
					self.pos_DTerm = pos_delta_error / pos_delta_time

			self.pos_last_time = self.pos_current_time
			self.pos_last_error = pos_error

			self.pos_output = self.pos_PTerm + (self.pos_Ki * self.pos_ITerm) + (self.pos_Kd * self.pos_DTerm)

		vel_error = self.pos_output - np.sqrt(x_vel*x_vel+y_vel*y_vel)
		self.vel_current_time = time.time()
		vel_delta_time = self.vel_current_time - self.vel_last_time
		vel_delta_error = vel_error - self.vel_last_error
		
		if (vel_delta_time >= self.sample_time):
			self.vel_PTerm = self.vel_Kp * vel_error
			self.vel_ITerm += vel_error * vel_delta_time

			if (self.vel_ITerm < -self.windup_guard):
				self.vel_ITerm = -self.windup_guard
			elif (self.vel_ITerm > self.windup_guard):
				self.vel_ITerm = self.windup_guard

			self.vel_DTerm = 0.0
			if vel_delta_time > 0:
				self.vel_DTerm = vel_delta_error / vel_delta_time

			self.vel_last_time = self.vel_current_time
			self.vel_last_error = vel_error

			self.vel_output = self.vel_PTerm + (self.vel_Ki * self.vel_ITerm) + (self.vel_Kd * self.vel_DTerm)


	def setWindup(self, windup):
		self.windup_guard = windup

	def setSampleTime(self, sample_time):
		self.sample_time = sample_time

def cb_vel(msg):
	global x_pos, x_vel, y_pos, y_vel
	# get pos, vel
	x_pos = -msg.pose.pose.position.y
	y_pos = msg.pose.pose.position.x
	x_vel = -msg.twist.twist.linear.y
	y_vel = msg.twist.twist.linear.x
def cb_ang(msg):
	global ang_left, ang_right
	ang_left = msg.left
	ang_right = msg.right
if __name__ == "__main__":
	rospy.init_node("pid_vel_node")
	rospy.Subscriber("/odometry/filtered", Odometry, cb_vel)
	rospy.Subscriber("/cmd_drive_ang", UsvDrive, cb_ang)
	pub_vel = rospy.Publisher("/cmd_drive", UsvDrive, queue_size = 20)
	pid = PID()
	pid.pos_SetPointx = 20
	pid.pos_SetPointy = 0
	while not rospy.is_shutdown():
		pid.update(x_pos, y_pos, x_vel, y_vel)
		msg = UsvDrive()
		out = pid.vel_output/5 
		if out > 0.7:
			out = 0.7
		if ang_left > 0.5 or ang_left < -0.5:
			out = out/5
		else:
			out = out*1.5
		msg.left = out + ang_left
		msg.right = out + ang_right
		pub_vel.publish(msg)
	
