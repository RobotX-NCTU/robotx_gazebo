#!/usr/bin/env python

'''
Author: Arthur Hsieh                                                             
Date: 2018/07/27                                                                
Last update: 2018/07/27                                                             
Hole ROI Extraction (Detect and Deliver)
Subscribe: 
  /zed_mid/rgb/image_rect_color/compressed 		(sensor_msgs/CompressedImage)
Publish:
  /hole/compressed 								(sensor_msgs/CompressedImage)
'''

import numpy as np
import cv2
from matplotlib import pyplot as plt
import roslib
import rospy
import copy
import sys
from sensor_msgs.msg import CompressedImage

class hole_detection():
	def __init__(self):
		self.image_mid_sub = rospy.Subscriber("/zed_mid/rgb/image_rect_color/compressed", CompressedImage, self.callback, queue_size = 1)
		self.image_pub = rospy.Publisher("/hole/compressed", CompressedImage, queue_size=5)
		self.np_arr = 0
		self.cv_img = 0
		self.lock = 0


	def callback(self, image_data):
		if self.lock == 0:
			self.np_arr = np.fromstring(image_data.data, np.uint8)
			self.cv_img = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)


	def process(self):
		if type(self.cv_img) == np.int:
			return
		self.lock = 1
		img = copy.copy(self.cv_img)
		self.lock = 0

		filtered_contours = self.get_white_board(img)
		img_rotated180 = img
		#find the white board contour
		for (cnt, box) in filtered_contours:
			x,y,w,h = box
			crop_img = img[y:y+h, x:x+w]
			hole_contours = self.get_hole_contours(crop_img)
			for (contour, boundBox) in hole_contours:
				x,y,w,h = boundBox
				cv2.rectangle(crop_img,(x,y),(x+w,y+h), (0,0,255), 10)

				#get image height, width
				height,width = crop_img.shape[:2]
				#calculate the center of the image
				center = (width/2, height/2)

				angle180 = 180
				scale = 1.0

				#perform the 180 degrees counter clockwise rotation holding at the center
				M = cv2.getRotationMatrix2D(center, angle180, scale)
				img_rotated180 = cv2.warpAffine(crop_img, M, (width, height))

		#### Create CompressedImage ####
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode('.jpg', img_rotated180)[1]).tostring()

		#publish new image
		self.image_pub.publish(msg)


	#get the white board
	def get_white_board(self, img):
		#convert to hsv
		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# define range of white color in HSV
		lower_white = np.array([0,0,0], dtype=np.uint8)
		upper_white = np.array([0,0,255], dtype=np.uint8)

		# Threshold the HSV image to get only white colors
		frame_threshed = cv2.inRange(hsv_img, lower_white, upper_white)
		
		# morphological opening
		kernel = np.ones((5,5), np.uint8)
		opening = cv2.morphologyEx(frame_threshed, cv2.MORPH_OPEN, kernel)

		# threshold
		ret,thresh = cv2.threshold(opening,30,255,0)

		filtered_contours = []

		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
		contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
		contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])

		#height,width = img.shape[:2]

		for (area,cnt) in contour_area:
			# plot box around contour
			x,y,w,h = cv2.boundingRect(cnt)
			box = (x,y,w,h)

			#remove lines
			if not(h>25 and w>25):
				continue
			#remove noise
			if cv2.contourArea(cnt)==0:
				continue
			#remove smaller boxes24462.0

			if not(area > 100000): 
				continue

			mask = np.zeros(thresh.shape,np.uint8)
			cv2.drawContours(mask,[cnt],0,255,-1)
			#mean_val = cv2.mean(img,mask = mask)
			filtered_contours.append( (cnt, box) )

		
		return filtered_contours



	def get_hole_contours(self, img):
		#convert to hsv
		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		BLACK = [np.array(x, np.uint8) for x in [[0,0,0], [180, 255, 125]] ]
		frame_threshed = cv2.inRange(hsv_img, BLACK[0], BLACK[1])

		# morphological opening
		kernel = np.ones((5,5), np.uint8)
		opening = cv2.morphologyEx(frame_threshed, cv2.MORPH_OPEN, kernel)

		# threshold
		ret,thresh = cv2.threshold(opening,36,255,0)

		filtered_contours = []

		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
		contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
		contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])

		height,width = img.shape[:2]

		for (area,cnt) in contour_area:
			# plot box around contour
			x,y,w,h = cv2.boundingRect(cnt)
			box = (x,y,w,h)
			aspect_ratio = float(h)/w

			#remove lower boxes
			if not(y>h/2):
				continue
			#remove lines
			if not(h>25 and w>25):
				continue
			#remove noise
			if cv2.contourArea(cnt)==0:
				continue
			#remove smaller boxes
			if not(area > 3000): 
				continue
			#remove nonsquare boxes
			if not(aspect_ratio < 1.2 and aspect_ratio > 0.8):
				continue

			mask = np.zeros(thresh.shape,np.uint8)
			cv2.drawContours(mask,[cnt],0,255,-1)
			filtered_contours.append( (cnt, box) )
		
		return filtered_contours



def main(args):
	ic = hole_detection()
	rospy.init_node('hole_detection', anonymous = True)        
	try:
		while (1):
			ic.process()
	except KeyboardInterrupt:
		print("Shutting down")



if __name__ == '__main__':
	main(sys.argv)