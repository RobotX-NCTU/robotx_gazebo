#!/usr/bin/env python
import sys
import rospy
from math import sqrt, acos

from robotx_gazebo.msg import ObstaclePose
from robotx_gazebo.msg import ObstaclePoseList
from robotx_gazebo.msg import UsvDrive


class drive_command_node():
	#initialize
	def __init__(self):
		self.sub = rospy.Subscriber("/obstacle_list", ObstaclePoseList, self.obstacle_cb)
		self.pub_drive = rospy.Publisher("/cmd_drive", UsvDrive, queue_size=5)
		self.drive = UsvDrive()
		self.Entrance = True
		self.Exit = False
		self.entrance_stop_x = 0.0
		self.entrance_stop_y = 0.0
		self.exit_stop_x = 0.0
		self.exit_stop_y = 0.0


	#callback function
	def obstacle_cb(self, pos_list):
		print("Receive callback")

		width = 100					#Width of the rec area
		length = 0					#Length of the rec area
		long_distance = 0		#Diagonal distance of the rec area
		entrance_x = 0			#middle point of two entrance buoys
		entrance_y = 0
		exit_x = 0					#middle point of two exit buoys
		exit_y = 0

		trans_x = 0					#transition point
		trans_y = 0
		group_1 = [[0 for x in range(2)] for y in range(2)]	#Entrance group 
		group_2 = [[0 for x in range(2)] for y in range(2)]	#Exit group
		diag = [[0 for x in range(2)] for y in range(2)]		#Diagonal group
		bot_to_closest = 0	#distance between the robot and the point that is the closest to it
		bot_to_width = 0   
		bot_to_length = 0
		length_angle = 0
		width_angle = 0

		#Four points are detected
		if (pos_list.size == 4):
			print("Four points are detected")
			dis_array = [0]*pos_list.size
			distance = 0

			#Store the distance between the first point and the remaining
			for i in range(1,pos_list.size):
				dis_array[i] = sqrt((pos_list.list[0].x - pos_list.list[i].x)*(pos_list.list[0].x - pos_list.list[i].x)  + (pos_list.list[0].y - pos_list.list[i].y)*(pos_list.list[0].y - pos_list.list[i].y)) 

			#Grouping points
			#group1 stores points that belong to entrance
			#group2 stores points that belong to exit
			for i in range(1,pos_list.size):
				distance = dis_array[i]
			  
				if(distance<=width):
					width = distance
					group_1[0][0] = pos_list.list[0].x
					group_1[0][1] = pos_list.list[0].y
					group_1[1][0] = pos_list.list[i].x
					group_1[1][1] = pos_list.list[i].y

					if(i==1):
						group_2[0][0] = pos_list.list[2].x
						group_2[0][1] = pos_list.list[2].y
						group_2[1][0] = pos_list.list[3].x
						group_2[1][1] = pos_list.list[3].y

					elif(i==2):
						group_2[0][0] = pos_list.list[1].x
						group_2[0][1] = pos_list.list[1].y
						group_2[1][0] = pos_list.list[3].x
						group_2[1][1] = pos_list.list[3].y

					else:
						group_2[0][0] = pos_list.list[1].x
						group_2[0][1] = pos_list.list[1].y
						group_2[1][0] = pos_list.list[2].x
						group_2[1][1] = pos_list.list[2].y

				if (distance>=long_distance):
					long_distance = distance
					diag[0][0] = pos_list.list[0].x
					diag[0][1] = pos_list.list[0].y
					diag[1][0] = pos_list.list[i].x
					diag[1][1] = pos_list.list[i].y

			length = sqrt(long_distance*long_distance - width*width) 

			####### Detect which side the robot is #######
			bot_to_closest = sqrt(group_1[0][0]*group_1[0][0] + group_1[0][1]*group_1[0][1])
			bot_to_width = sqrt(group_1[1][0]*group_1[1][0] + group_1[1][1]*group_1[1][1])
			bot_to_length = sqrt(group_2[0][0]*group_2[0][0] + group_2[0][1]*group_2[0][1])

			width_val = (bot_to_closest*bot_to_closest + width*width - bot_to_width*bot_to_width) / (2*bot_to_closest*width)
			length_val = (bot_to_closest*bot_to_closest + length*length - bot_to_length*bot_to_length) / (2*bot_to_closest*length)
			
			#Check the value
			if width_val > 1:
				width_val = 1
			elif width_val < -1:
				width_val = -1

			if length_val > 1:
				length_val = 1
			elif length_val < -1:
				length_val = -1.

			width_angle = acos(width_val) * 180 / 3.141596
			length_angle = acos(length_val) * 180 / 3.141596

			if (length_angle >= 90):
				self.Entrance = True  #Entrance side
			elif (width_angle >= 90 and length_angle <= 90):
				self.Entrance = False #Length side

			#Calculate middle points between two entrance and exit points, respectively
			entrance_x = (group_1[0][0] + group_1[1][0]) / 2
			entrance_y = (group_1[0][1] + group_1[1][1]) / 2
			exit_x = (group_2[0][0] + group_2[1][0]) / 2
			exit_y = (group_2[0][1] + group_2[1][1]) / 2

			#Calculate points that are in front of two middle points
			self.entrance_stop_x = entrance_x - (exit_x - entrance_x) / 6
			self.entrance_stop_y = entrance_y - (exit_y - entrance_y) / 6
			self.exit_stop_x = exit_x + (exit_x - entrance_x) / 6
			self.exit_stop_y = exit_y + (exit_y - entrance_y) / 6

			#Calculate transition point
			trans_x = diag[0][0] - (diag[1][0] - diag[0][0]) / 5
			trans_y = diag[0][1] - (diag[1][1] - diag[0][1]) / 5

			if (self.Exit):
				self.drive_to_exit(pos_list, self.exit_stop_x, self.exit_stop_y)
			elif (self.Entrance):
				self.drive_to_entrance(pos_list, self.entrance_stop_x, self.entrance_stop_y, self.exit_stop_x, self.exit_stop_y)
			else:
				self.drive_to_trans(pos_list, trans_x, trans_y, self.entrance_stop_x, self.entrance_stop_y)

		#If three points are detected
		elif (pos_list.size == 3):
			print("Three points are detected")

			if (self.Entrance and self.Exit):
				self.exit_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2
				self.exit_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2

				self.drive_to_exit(pos_list, self.exit_stop_x, self.exit_stop_y)
			
			elif (self.Entrance):
				self.find_normal_point(pos_list)
				self.drive_to_entrance(pos_list, self.entrance_stop_x, self.entrance_stop_y, self.exit_stop_x, self.exit_stop_y)

			else:
				self.entrance_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2
				self.entrance_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2

				trans_x = pos_list.list[0].x - (pos_list.list[2].x - pos_list.list[0].x) / 7
				trans_y = pos_list.list[0].y - (pos_list.list[2].y - pos_list.list[0].y) / 7
				self.drive_to_trans(pos_list, trans_x, trans_y, self.entrance_stop_x, self.entrance_stop_y)

		#Less than four but more than two points are detected
		elif (pos_list.size == 2):
			print("Two points are detected")
			if (self.Entrance and self.Exit):
				self.exit_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2
				self.exit_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2

				self.drive_to_exit(pos_list, self.exit_stop_x, self.exit_stop_y)

			elif (self.Entrance):
				self.find_normal_point(pos_list)
				self.drive_to_entrance(pos_list, self.entrance_stop_x, self.entrance_stop_y, self.exit_stop_x, self.exit_stop_y)

			else:
				self.entrance_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2
				self.entrance_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2

				trans_x = pos_list.list[0].x - (pos_list.list[1].x - pos_list.list[0].x) / 2
				trans_y = pos_list.list[0].y - (pos_list.list[1].y - pos_list.list[0].y) / 2

				self.drive_to_trans(pos_list, trans_x, trans_y, self.entrance_stop_x, self.entrance_stop_y)

		#If less than two points or more than four points are detected
		else:
			print("Go Straight")
			self.drive.right = 0.3
			self.drive.left  = 0.3
			self.pub_drive.publish(self.drive)



	#Find the point on the normal line between two buoys
	def find_normal_point(self, pos_list):
		point1 = [0] * 2
		point2 = [0] * 2
		s = -(pos_list.list[0].x - pos_list.list[1].x) / (pos_list.list[0].y - pos_list.list[1].y) #slope of the normal

		point1[0] = ((pos_list.list[0].x + pos_list.list[1].x) / 2) - 1.5
		point1[1] = ((pos_list.list[0].y + pos_list.list[1].y) / 2) - 1.5 * s

		point2[0] = ((pos_list.list[0].x + pos_list.list[1].x) / 2) + 1.5
		point2[1] = ((pos_list.list[0].y + pos_list.list[1].y) / 2) + 1.5 * s

		#Find closer point
		if (abs(point1[0]) < abs(point2[0])):
			self.entrance_stop_x = point1[0]
			self.entrance_stop_y = point1[1]
			self.exit_stop_x = point2[0]
			self.exit_stop_y = point2[1]

		else:
			self.entrance_stop_x = point2[0]
			self.entrance_stop_y = point2[1]
			self.exit_stop_x = point1[0]
			self.exit_stop_y = point1[1]



	#Drive to entrance point
	def drive_to_entrance(self, pos_list, entrance_stop_x, entrance_stop_y, exit_stop_x, exit_stop_y):
		print("Receive entrance func")

		if (abs(entrance_stop_x) < 1.5 and abs(entrance_stop_y) < 0.5):
			print("Drive to exit")
			self.Exit = True
			self.drive_to_exit(pos_list, exit_stop_x, exit_stop_y)
	  
		elif (abs(entrance_stop_x) < 1 and entrance_stop_y > 0.5):
			print("Go straight")
			self.drive.right = 0.4
			self.drive.left  = 0.4
			self.pub_drive.publish(self.drive)
	  
		elif ((entrance_stop_x < 0) and ((entrance_stop_y) > 0.5)):
			print("Left turn")
			self.drive.right = 0.3
			self.drive.left  = 0.1
			self.pub_drive.publish(self.drive)

		elif ((entrance_stop_x > 0) and ((entrance_stop_y) > 0.5)):
			print("Right turn")
			self.drive.right = 0.1
			self.drive.left  = 0.3
			self.pub_drive.publish(self.drive)

		else:
			self.Exit = True
			self.drive.right = 0.4
			self.drive.left  = 0.4
			self.pub_drive.publish(self.drive)


	#Drive to transition point
	def drive_to_trans(self, pos_list, trans_x, trans_y, entrance_stop_x, entrance_stop_y):
		print("Receive transition func")

		if (abs(trans_x) < 1) and (abs(trans_y) < 0.5):
			print("Drive to entrance")
			self.Entrance = True
			self.drive_to_entrance(pos_list, entrance_stop_x, entrance_stop_y, 0, 0)

		elif (abs(trans_x) < 1) and (abs(trans_y) > 0.5):
			print("Go straight")
			self.drive.right = 0.4
			self.drive.left  = 0.4
			self.pub_drive.publish(self.drive)

		elif ((trans_x < 0) and ((trans_y) > 0)):
			print("Left turn")
			self.drive.right = 0.3
			self.drive.left  = 0.1
			self.pub_drive.publish(self.drive)

		elif ((trans_x > 0) and ((trans_y) > 0)):
			print("Right turn")
			self.drive.right = 0.1
			self.drive.left  = 0.3
			self.pub_drive.publish(self.drive)

		else:
			#self.Entrance = True
			self.drive.right = 0.4
			self.drive.left  = 0.4
			self.pub_drive.publish(self.drive)


	#Drive to exit point
	def drive_to_exit(self, pos_list, exit_stop_x, exit_stop_y):
		print("Receive exit func")

		if (abs(exit_stop_x) < 0.5) and ((exit_stop_y) < 0.5):
			print("Arrive")
			self.drive.right = 0.4;
			self.drive.left  = 0.4;

		elif (abs(exit_stop_x) < 0.5) and ((exit_stop_y) > 0.5):
			print("Go straight")
			self.drive.right = 0.4;
			self.drive.left  = 0.4;

		elif ((exit_stop_x < 0) and ((exit_stop_y) > 0)):
			print("Left turn")
			self.drive.right = 0.3;
			self.drive.left  = 0.1;

		elif ((exit_stop_x > 0) and ((exit_stop_y) > 0)):
			print("Right turn")
			self.drive.right = 0.1;
			self.drive.left  = 0.3;

		self.pub_drive.publish(self.drive);



def main(args):
	rospy.init_node('drive_command_node', anonymous = True) 
	ic = drive_command_node()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
