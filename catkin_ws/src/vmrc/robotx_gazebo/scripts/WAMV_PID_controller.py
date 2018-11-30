#!/usr/bin/env python

import time
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from robotx_gazebo.msg import UsvDrive
from robotx_gazebo.srv import waypoint
from std_msgs.msg import Int32
from std_srvs.srv import *
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
start_flag = 0
station_keep_flag = 0
x_pos = 0.0000001	# to prevent divid by zero
y_pos = 0
x_vel = 0
y_vel = 0
yaw = 0
last_time = -1
last_x_pos = -1
last_y_pos = -1
linear_vel_const = 0.3
waypoint_index = 0
waypoints = None
new_waypoints = None
stable = 0
lock = 0
State = 0
class pos_vel_PID:
	

	def __init__(self, pos_P=0.7, pos_I=0.005, pos_D=0.2, vel_P=0.4, vel_I=0.0, vel_D=0.0):
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

	def update(self, x_pos, y_pos, x_vel, y_vel, yaw):
		global waypoints, waypoint_index, station_keep_flag, stable
		

		self.pos_SetPointx = (waypoints[waypoint_index][0])
		self.pos_SetPointy = (waypoints[waypoint_index][1])

		pos_error = np.sqrt((self.pos_SetPointx - x_pos)*(self.pos_SetPointx - x_pos)+(self.pos_SetPointy - y_pos)*(self.pos_SetPointy - y_pos))
		#print pos_error, " ", x_pos, " ", self.pos_SetPointx, " ", y_pos, " ", self.pos_SetPointy
		error = np.arctan2((self.pos_SetPointy - y_pos),(self.pos_SetPointx - x_pos)) - yaw
		if np.abs(error)> np.pi:
			if error > 0:
				error = error-2*np.pi
			else:
				error = error+2*np.pi

		if station_keep_flag == 0:
			'''
			if np.sqrt((self.pos_SetPointx - x_pos)*(self.pos_SetPointx - x_pos)+(self.pos_SetPointy - y_pos)*(self.pos_SetPointy - y_pos)) < 3:
				self.vel_output = 0
				print "stop"

			else:
				self.vel_output = linear_vel_const
				print "const vel"
			'''
			if pos_error > (3+0.2):
				self.vel_output = linear_vel_const
			else:
				self.vel_output = 0
			return 0

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


		vel_error = self.pos_output - np.sqrt(x_vel*x_vel + y_vel*y_vel)
		print vel_error, self.pos_output, np.sqrt(x_vel*x_vel + y_vel*y_vel)
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
			if pos_error < 1.5:
				self.vel_output = 0				
				if np.sqrt(x_vel*x_vel+y_vel*y_vel) < 0.1:
					stable = 1
			if np.abs(error) > np.pi/2:
				print "reverse"
				self.vel_output = -self.vel_output
				if np.abs(error) > np.pi*3/4:
					self.vel_output = self.vel_output * 0.3
			if np.abs(error) < np.pi/2 and np.abs(error) > np.pi/4:
				self.vel_output = self.vel_output * 0.3

			


	def setWindup(self, windup):
		self.windup_guard = windup

	def setSampleTime(self, sample_time):
		self.sample_time = sample_time

class ang_PID:
	
	def __init__(self, P=0.3, I=0.01, D=0.4):
		self.wait_flag = 0
		self.wait_start = 0
		self.Kp = P
		self.Ki = I
		self.Kd = D

		self.sample_time = 0.00
		self.current_time = time.time()
		self.last_time = self.current_time

		self.lock_aux_pointx = 0
		self.lock_aux_pointy = 0
		self.pub_aux = rospy.Publisher("/aux_marker", Marker, queue_size = 10)
		self.pub_tgwp = rospy.Publisher("/target_waypoint", Marker, queue_size = 10)
		self.pub_lwp = rospy.Publisher("/last_waypoint", Marker, queue_size = 10)
		self.clear()

	def clear(self):
		self.pos_SetPointx = 0.0
		self.pos_SetPointy = 0.0
		self.SetPointx = 0.0
		self.SetPointy = 0.0
		self.PTerm = 0.0
		self.ITerm = 0.0
		self.DTerm = 0.0
		self.last_error = 0.0

		# Windup Guard
		self.int_error = 0.0
		self.windup_guard = 20.0

		self.output = 0.0

	def update(self, x_pos, y_pos, yaw):
		global waypoints, waypoint_index, station_keep_flag, stable, lock
		
		self.pos_SetPointx = waypoints[waypoint_index][0]
		self.pos_SetPointy = waypoints[waypoint_index][1]
		# get waypoint index
		last_waypoint_index = waypoint_index
		for i in range(waypoints.shape[0]):
			if np.sqrt((waypoints[i][0] - x_pos)*(waypoints[i][0] - x_pos) + (waypoints[i][1] - y_pos)*(waypoints[i][1] - y_pos)) < 3:
				if i >= last_waypoint_index:
					if i == waypoints.shape[0]-1:
						waypoint_index = i
					else:
						waypoint_index = i + 1
		
		if waypoint_index == waypoints.shape[0]-1 and np.sqrt((waypoints[waypoint_index][0] - x_pos)*(waypoints[waypoint_index][0] - x_pos) + (waypoints[waypoint_index][1] - y_pos)*(waypoints[waypoint_index][1] - y_pos)) < 3:
			station_keep_flag = 1

		if time.time() - self.wait_start > 10:
			self.wait_flag = 0
		#print "delta t", time.time() - self.wait_start
		pos_error = np.sqrt((self.pos_SetPointx - x_pos)*(self.pos_SetPointx - x_pos)+(self.pos_SetPointy - y_pos)*(self.pos_SetPointy - y_pos))
		waypoint_error = np.sqrt((waypoints[waypoint_index][0] - x_pos)*(waypoints[waypoint_index][0] - x_pos)+(waypoints[waypoint_index][1] - y_pos)*(waypoints[waypoint_index][1] - y_pos))
		if np.abs(waypoint_error) < 3 and station_keep_flag == 1 and stable == 1:
			print "turn to desired yaw"
			# lock yaw
			if waypoints[waypoint_index][2] != -10:
				error = waypoints[waypoint_index][2] - yaw
			else:
				error = np.arctan2((waypoints[waypoint_index][1] - waypoints[waypoint_index-1][1]),(waypoints[waypoint_index][0] - waypoints[waypoint_index-1][0])) - yaw
			
		else:
			error = np.arctan2((self.SetPointy - y_pos),(self.SetPointx - x_pos)) - yaw
		

		if waypoint_index >= 1:
			last_waypointx = waypoints[waypoint_index-1][0]
			last_waypointy = waypoints[waypoint_index-1][1]
		else:
			if self.lock_aux_pointx == 0:
				self.lock_aux_pointx = x_pos
				self.lock_aux_pointy = y_pos
			last_waypointx = self.lock_aux_pointx
			last_waypointy = self.lock_aux_pointy

		if np.abs(waypoints[waypoint_index][0]-last_waypointx) < 1:
			
			dist_to_line = np.abs(x_pos-last_waypointx)
			tmp_x = last_waypointx
			tmp_y = y_pos

			dir_vectorx = 0
			dir_vectory = 1


		else:
			line_slope = (waypoints[waypoint_index][1]-last_waypointy)/(waypoints[waypoint_index][0]-last_waypointx)
			line_intercept = waypoints[waypoint_index][1]-line_slope*waypoints[waypoint_index][0]
			dist_to_line = np.abs(line_slope*x_pos + line_intercept - y_pos)/np.sqrt(line_slope*line_slope+1)

			tmp_x = x_pos + (line_slope/np.sqrt(line_slope*line_slope+1))*dist_to_line
			tmp_y = y_pos - dist_to_line
			#print line_slope, line_intercept, dist_to_line
			if np.abs(line_slope*tmp_x + line_intercept - tmp_y)/np.sqrt(line_slope*line_slope+1) > dist_to_line:
				tmp_x = x_pos - (line_slope/np.sqrt(line_slope*line_slope+1))*dist_to_line
				tmp_y = y_pos + dist_to_line

			dir_vectorx = 1/np.sqrt(line_slope*line_slope+1)
			dir_vectory = line_slope/np.sqrt(line_slope*line_slope+1)
		#print dir_vectorx, dir_vectory
		#print dist_to_line
		proc_dist = np.sqrt(3*3-dist_to_line*dist_to_line)

		aux_pointx = tmp_x + dir_vectorx * proc_dist
		aux_point2x = tmp_x + 2 * dir_vectorx * proc_dist
		aux_pointy = tmp_y + dir_vectory * proc_dist
		aux_point2y = tmp_y + 2 * dir_vectory * proc_dist
		if np.abs((waypoints[waypoint_index][0] - tmp_x) * (waypoints[waypoint_index][0] - tmp_x) + (waypoints[waypoint_index][1] - tmp_y) * (waypoints[waypoint_index][1] - tmp_y)) < np.abs((waypoints[waypoint_index][0] - aux_pointx) * (waypoints[waypoint_index][0] - aux_pointx) + (waypoints[waypoint_index][1] - aux_pointy) * (waypoints[waypoint_index][1] - aux_pointy)):
			aux_pointx = tmp_x - dir_vectorx * proc_dist
			aux_point2x = tmp_x - 2 * dir_vectorx * proc_dist
			aux_pointy = tmp_y - dir_vectory * proc_dist
			aux_point2y = tmp_y - 2 * dir_vectory * proc_dist
		#print np.sqrt((tmp_x - x_pos)*(tmp_x - x_pos)+(tmp_y - y_pos)*(tmp_y - y_pos))
		if np.sqrt((tmp_x - x_pos)*(tmp_x - x_pos)+(tmp_y - y_pos)*(tmp_y - y_pos)) < 3 :
			
			
			if station_keep_flag == 1 and pos_error < 3:
				if lock == 0:
					self.SetPointx = aux_point2x
					self.SetPointy = aux_point2y
					aux_pointx = aux_point2x
					aux_pointy = aux_point2y
					lock = 1
				aux_pointx = self.SetPointx
				aux_pointy = self.SetPointy
				
			else:
				self.SetPointx = aux_pointx
				self.SetPointy = aux_pointy
				lock = 0

			#print "aux", aux_pointx, aux_pointy

			wpoints = []
			for i in range(1):
				p = Point()
				p.x = aux_pointx
				p.y = aux_pointy
				p.z = 0
				wpoints.append(p)
			marker = Marker()
			marker.header.frame_id = "/odom"

			marker.type = marker.POINTS
			marker.action = marker.ADD
			marker.pose.orientation.w = 1

			marker.points = wpoints;
			t = rospy.Duration()
			marker.lifetime = t
			marker.scale.x = 0.4
			marker.scale.y = 0.4
			marker.scale.z = 0.4
			marker.color.a = 1.0
			marker.color.r = 0
			marker.color.g = 0
			marker.color.b = 1.0

			self.pub_aux.publish(marker)

			
		else:
			lock = 0
			self.SetPointx = tmp_x
			self.SetPointy = tmp_y
			print "tmp", tmp_x, tmp_y
			
			wpoints = []
			for i in range(1):
				p = Point()
				p.x = tmp_x
				p.y = tmp_y
				p.z = 0
				wpoints.append(p)
			marker = Marker()
			marker.header.frame_id = "/odom"

			marker.type = marker.POINTS
			marker.action = marker.ADD
			marker.pose.orientation.w = 1

			marker.points = wpoints;
			t = rospy.Duration()
			marker.lifetime = t
			marker.scale.x = 0.4
			marker.scale.y = 0.4
			marker.scale.z = 0.4
			marker.color.a = 1.0
			marker.color.r = 0
			marker.color.g = 1.0
			marker.color.b = 0

			self.pub_aux.publish(marker)


		wpoints = []
		for i in range(1):
			p = Point()
			p.x = waypoints[waypoint_index][0]
			p.y = waypoints[waypoint_index][1]
			p.z = 0
			wpoints.append(p)
		marker2 = Marker()
		marker2.header.frame_id = "/odom"

		marker2.type = marker.POINTS
		marker2.action = marker.ADD
		marker2.pose.orientation.w = 1

		marker2.points = wpoints;
		t = rospy.Duration()
		marker2.lifetime = t
		marker2.scale.x = 0.4
		marker2.scale.y = 0.4
		marker2.scale.z = 0.4
		marker2.color.a = 1.0
		marker2.color.r = 0.5
		marker2.color.g = 0
		marker2.color.b = 0.5

		self.pub_tgwp.publish(marker2)

		wpoints = []
		for i in range(1):
			p = Point()
			p.x = last_waypointx
			p.y = last_waypointy
			p.z = 0
			wpoints.append(p)
		marker2 = Marker()
		marker2.header.frame_id = "/odom"

		marker2.type = marker.POINTS
		marker2.action = marker.ADD
		marker2.pose.orientation.w = 1

		marker2.points = wpoints;
		t = rospy.Duration()
		marker2.lifetime = t
		marker2.scale.x = 0.4
		marker2.scale.y = 0.4
		marker2.scale.z = 0.4
		marker2.color.a = 1.0
		marker2.color.r = 0
		marker2.color.g = 0.5
		marker2.color.b = 0.5

		self.pub_lwp.publish(marker2)


		'''
		#print np.abs(waypoint_error)
		if np.abs(waypoint_error) < 5 and waypoints is not None:
			if waypoint_index < waypoints.shape[0]-1:
				if self.wait_flag == 0 and station_keep_flag == 0:
					waypoint_index = waypoint_index + 1
					self.wait_flag = 1
					self.wait_start = time.time()
					print "target waypoint", waypoint_index
			#print waypoint_index, waypoints.shape[0]
			else:
				station_keep_flag = 1
				print "station keep in last waypoint"
		'''
		#print "error", np.arctan((self.pos_SetPointy - y_pos)/(self.pos_SetPointx - x_pos)) - yaw
		if np.abs(error)> np.pi:
			if error > 0:
				error = error-2*np.pi
			else:
				error = error+2*np.pi
		if station_keep_flag == 1 and np.abs(waypoint_error) > 3:
			if np.abs(error) > np.pi/2:
				if error < 0:
					error = error + np.pi
				else:
					error = error - np.pi
				
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
	global new_waypoints
	print "add waypoint:", req.waypointx, req.waypointy
	if new_waypoints is None:
		new_waypoints = np.array([[req.waypointx, req.waypointy, req.yaw]])
	else:
		new_waypoints = np.vstack((new_waypoints, np.array([req.waypointx, req.waypointy, req.yaw])))
	return new_waypoints.shape[0]
def add_current_loc_handler(req):
	global x_pos, y_pos, yaw	
	global new_waypoints
	if new_waypoints is None:
		new_waypoints = np.array([[x_pos, y_pos, yaw]])
	else:
		new_waypoints = np.vstack((new_waypoints, np.array([x_pos, y_pos, yaw])))
	res = TriggerResponse()
	res.success = True
	res.message = "waypoint nav started"
	return res
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
	global new_waypoints, waypoint_index, start_flag, station_keep_flag, lock
	print "clear waypoints"
	new_waypoints = None
	waypoint_index = 0
	station_keep_flag = 0
	start_flag = 0
	lock = 0
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
	global station_keep_flag, waypoint_index
	print "station keep unlocked"
	station_keep_flag = 0
	waypoint_index = waypoint_index + 1
	res = TriggerResponse()
	res.success = True
	res.message = "station keep unlocked"
	return res


def cb_nav(msg):
	global x_pos, x_vel, y_pos, y_vel, yaw, last_time, last_x_pos, last_y_pos
	# get pos, vel
	x_pos = msg.pose.pose.position.x
	y_pos = msg.pose.pose.position.y
	if last_time != -1:
		delta_t = time.time() - last_time		
		x_vel = (x_pos - last_x_pos)/delta_t
		y_vel = (y_pos - last_y_pos)/delta_t
	last_time = time.time()
	#print x_vel, y_vel
	last_x_pos = x_pos
	last_y_pos = y_pos
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
	add_current_loc_srv = rospy.Service("/add_current_loc", Trigger, add_current_loc_handler)	
	pub_marker = rospy.Publisher("/waypoint_marker", Marker, queue_size = 10)
	pub_wp_nav_state = rospy.Publisher("/wp_nav_state", Int32, queue_size = 10)
	print "waiting for start srv"
	while start_flag == 0 and waypoints is None:	
		rospy.sleep(0.1)
	
	waypoints = new_waypoints
	pos_vel_pid = pos_vel_PID()
	pos_vel_pid.pos_SetPointx = waypoints[waypoint_index][0]
	pos_vel_pid.pos_SetPointy = waypoints[waypoint_index][1]
	
	ang_pid = ang_PID()
	ang_pid.pos_SetPointx = waypoints[waypoint_index][0]
	ang_pid.pos_SetPointy = waypoints[waypoint_index][1]

	while not rospy.is_shutdown():
		rospy.sleep(0.1)
		waypoints = new_waypoints
		wpoints = []
		while waypoints is None:
			print "no waypoints"
			ang_pid.lock_aux_pointx = 0
			waypoints = new_waypoints
		for i in range(waypoints.shape[0]):
			p = Point()
			p.x = waypoints[i][0]
			p.y = waypoints[i][1]
			p.z = 0
			wpoints.append(p)
		marker = Marker()
		marker.header.frame_id = "/odom"

		marker.type = marker.POINTS
		marker.action = marker.ADD
		marker.pose.orientation.w = 1

		marker.points = wpoints;
		t = rospy.Duration()
		marker.lifetime = t
		marker.scale.x = 0.4
		marker.scale.y = 0.4
		marker.scale.z = 0.4
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 1.0

		pub_marker.publish(marker)
		pos_vel_pid.update(x_pos, y_pos, x_vel, y_vel, yaw)
		ang_pid.update(x_pos, y_pos, yaw)
		pub_wp_nav_state.publish(station_keep_flag)
		
		out = pos_vel_pid.vel_output
		print "linear: ", out, "angular: ", ang_pid.output
		if out > 1:
			out = 1


		if abs(ang_pid.output) < 0.05:
			ang_pid.output = 0
		ang_left = -ang_pid.output
		ang_right = ang_pid.output

		if abs(ang_left) > 1:
			out = 0
		else:
			out = out*(1-abs(ang_left))*(1-abs(ang_left))

		if abs(ang_left) > 0.6:
			ang_left = (ang_left/abs(ang_left))*0.6
			out = 0
		if abs(ang_right) > 0.6:
			ang_right = (ang_right/abs(ang_right))*0.6
			out = 0

		#print "linear: ", out, "angular: ", ang_left
		#print waypoints
		#print waypoint_index
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
		
