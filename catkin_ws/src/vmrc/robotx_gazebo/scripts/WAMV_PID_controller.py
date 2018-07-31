#!/usr/bin/env python

import time
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from robotx_gazebo.msg import UsvDrive
from robotx_gazebo.srv import waypoint
from std_srvs.srv import *
import tf
start_flag = 0
station_keep_flag = 0
x_pos = 0.0000001	# to prevent divid by zero
y_pos = 0
x_vel = 0
y_vel = 0
yaw = 0

waypoint_index = 0
waypoints = None

class pos_vel_PID:
	

	def __init__(self, pos_P=0.2, pos_I=0.01, pos_D=0.2, vel_P=0.2, vel_I=0.0, vel_D=0.2):
		self.wait_flag = 0
		self.wait_start = 0
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
		global waypoints, waypoint_index, station_keep_flag
		if time.time() - self.wait_start > 5:
			self.wait_flag = 0
		pos_error = np.sqrt((self.pos_SetPointx - x_pos)*(self.pos_SetPointx - x_pos)+(self.pos_SetPointy - y_pos)*(self.pos_SetPointy - y_pos))
		#print pos_error, " ", x_pos, " ", self.pos_SetPointx, " ", y_pos, " ", self.pos_SetPointy
		if np.abs(pos_error) < 3:

			if waypoint_index < waypoints.shape[0]-1:
				if self.wait_flag == 0 and station_keep_flag == 0:
					waypoint_index = waypoint_index + 1
					self.wait_flag = 1
					self.wait_start = time.time()
					print "target waypoint", waypoint_index
					self.pos_SetPointx = (waypoints[waypoint_index][0])
					self.pos_SetPointy = (waypoints[waypoint_index][1])
			
			pos_error = 0

		#print pos_error
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

class ang_PID:
	
	def __init__(self, P=0.3, I=0.005, D=0.05):
		self.wait_flag = 0
		self.wait_start = 0
		self.Kp = P
		self.Ki = I
		self.Kd = D

		self.sample_time = 0.00
		self.current_time = time.time()
		self.last_time = self.current_time

		self.clear()

	def clear(self):
		self.pos_SetPointx = 0.0
		self.pos_SetPointy = 0.0

		self.PTerm = 0.0
		self.ITerm = 0.0
		self.DTerm = 0.0
		self.last_error = 0.0

		# Windup Guard
		self.int_error = 0.0
		self.windup_guard = 20.0

		self.output = 0.0

	def update(self, x_pos, y_pos, yaw):
		global waypoints, waypoint_index
		#print "delta t", time.time() - self.wait_start
		error = np.arctan2((self.pos_SetPointy - y_pos),(self.pos_SetPointx - x_pos)) - yaw
		pos_error = np.sqrt((self.pos_SetPointx - x_pos)*(self.pos_SetPointx - x_pos)+(self.pos_SetPointy - y_pos)*(self.pos_SetPointy - y_pos))
		if np.abs(pos_error) < 2:        
			
			self.pos_SetPointx = (waypoints[waypoint_index][0])
			self.pos_SetPointy = (waypoints[waypoint_index][1])
			
			error = 0

		#print "error", np.arctan((self.pos_SetPointy - y_pos)/(self.pos_SetPointx - x_pos)) - yaw
		if np.abs(error)> np.pi:
			if error > 0:
				error = error-2*np.pi
			else:
				error = error+2*np.pi

		#print "error", error
		#print "yaw", yaw
		#print "target", np.arctan2((self.pos_SetPointy - y_pos), (self.pos_SetPointx - x_pos))
		self.current_time = time.time()
		delta_time = self.current_time - self.last_time
		delta_error = error - self.last_error

		if (delta_time >= self.sample_time):
			self.PTerm = self.Kp * error
			self.ITerm += error * delta_time

			if (self.ITerm < -self.windup_guard):
				self.ITerm = -self.windup_guard
			elif (self.ITerm > self.windup_guard):
				self.ITerm = self.windup_guard

			self.DTerm = 0.0
			if delta_time > 0:
				self.DTerm = delta_error / delta_time

			self.last_time = self.current_time
			self.last_error = error

			self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

	def setKp(self, proportional_gain):
		self.Kp = proportional_gain

	def setKi(self, integral_gain):
		self.Ki = integral_gain

	def setKd(self, derivative_gain):
		self.Kd = derivative_gain

	def setWindup(self, windup):
		self.windup_guard = windup

	def setSampleTime(self, sample_time):
		self.sample_time = sample_time

def add_waypoint_handler(req):
	global waypoints
	print "add waypoint:", req.waypointx, req.waypointy
	if waypoints is None:
		waypoints = np.array([[req.waypointx, req.waypointy]])
	else:
		waypoints = np.vstack((waypoints, np.array([req.waypointx, req.waypointy])))
	return waypoints.shape[0]

def start_waypoint_handler(req):
	global start_flag
	print "start waypoints nav"
	start_flag = 1
	res = TriggerResponse()
	res.success = True
	res.message = "waypoint nav started"
	return res

def pause_waypoint_handler(req):
	global start_flag
	print "pause waypoints nav"
	start_flag = 0
	res = TriggerResponse()
	res.success = True
	res.message = "waypoint nav stopped"
	return res

def clear_waypoints_handler(req):
	global waypoints, waypoint_index
	print "clear waypoints"
	waypoints = None
	waypoint_index = 0
	res = TriggerResponse()
	res.success = True
	res.message = "waypoints flushed"
	return res

def station_keep_handler(req):
	global station_keep_flag, waypoint_index
	print "station keep on ", waypoint_index, "waypoint"
	station_keep_flag = 1
	res = TriggerResponse()
	res.success = True
	res.message = "station keep"
	return res

def station_keep_unlock_handler(req):
	global station_keep_flag
	print "station keep unlocked"
	station_keep_flag = 0
	res = TriggerResponse()
	res.success = True
	res.message = "station keep unlocked"
	return res


def cb_nav(msg):
	global x_pos, x_vel, y_pos, y_vel, yaw
	# get pos, vel
	x_pos = msg.pose.pose.position.x
	y_pos = msg.pose.pose.position.y
	x_vel = msg.twist.twist.linear.x
	y_vel = msg.twist.twist.linear.y
	quaternion = (
	msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y,
	msg.pose.pose.orientation.z,
	msg.pose.pose.orientation.w)
	_, _, yaw = tf.transformations.euler_from_quaternion(quaternion)


if __name__ == "__main__":
	rospy.init_node("wamv_pid_node")
	rospy.Subscriber("/odometry/filtered", Odometry, cb_nav)
	pub_vel = rospy.Publisher("/cmd_drive", UsvDrive, queue_size = 20)
	start_srv = rospy.Service("/start_waypoint_nav", Trigger, start_waypoint_handler)
	stop_srv = rospy.Service("/pause_waypoint_nav", Trigger, pause_waypoint_handler)
	station_keep_srv = rospy.Service("/station_keep", Trigger, station_keep_handler)
	station_keep_srv = rospy.Service("/station_keep_unlock", Trigger, station_keep_unlock_handler)
	clear_waypoints_srv = rospy.Service("/clear_waypoints", Trigger, clear_waypoints_handler)
	add_waypoint_srv = rospy.Service("/add_waypoint", waypoint, add_waypoint_handler)

	print "waiting for start srv"
	while start_flag == 0 and waypoints is None:	
		rospy.sleep(0.1)

	pos_vel_pid = pos_vel_PID()
	pos_vel_pid.pos_SetPointx = waypoints[waypoint_index][0]
	pos_vel_pid.pos_SetPointy = waypoints[waypoint_index][1]
	
	ang_pid = ang_PID()
	ang_pid.pos_SetPointx = waypoints[waypoint_index][0]
	ang_pid.pos_SetPointy = waypoints[waypoint_index][1]

	while not rospy.is_shutdown():
		pos_vel_pid.update(x_pos, y_pos, x_vel, y_vel)
		ang_pid.update(x_pos, y_pos, yaw)

		
		out = pos_vel_pid.vel_output
		#print "linear: ", out, "angular: ", ang_pid.output
		if out > 0.3:
			out = 0.3


		if abs(ang_pid.output) < 0.05:
			ang_pid.output = 0
		ang_left = -ang_pid.output
		ang_right = ang_pid.output

		if abs(ang_left) > 0.3:
			out = 0
		else:
			out = out

		if abs(ang_left) > 0.3:
			ang_left = (ang_left/abs(ang_left))*0.3
		if abs(ang_right) > 0.3:
			ang_right = (ang_right/abs(ang_right))*0.3

		print "linear: ", out, "angular: ", ang_left
		if start_flag == 0:
			msg = UsvDrive()
			msg.left = 0
			msg.right = 0
			pub_vel.publish(msg)
		else:
			# publish UsvDrive message
			msg = UsvDrive()
			msg.left = out + ang_left
			msg.right = out + ang_right
			pub_vel.publish(msg)
		
