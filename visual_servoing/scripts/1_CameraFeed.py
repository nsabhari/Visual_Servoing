#!/usr/bin/env python

import rospy, roslib, sys, cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from std_srvs.srv import Empty as EmptySrv
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

class scaraCam:
	def __init__(self):
		self.bridge = CvBridge()
		self.imgRGB = rospy.Subscriber("/scaraCam/color/image_raw",Image,self.callback_imgRGB)
		self.GazRGB = rospy.Subscriber("/gazeboCam/gazeboCam/rgb",Image,self.callback_gazRGB)
		self.recording = rospy.Subscriber("/camRecorder", Int32, self.callback_record)
		self.frame = np.zeros((480,640,3), np.uint8)
		self.gazFrame = np.zeros((480,640,3), np.uint8)
		self.frameCubes = np.zeros((480,640,3), np.uint8)
		self.display = np.zeros((480,640,3), np.uint8)
		self.flagRecord = 0
		self.out = 0

	def callback_imgRGB(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data,"passthrough")
			self.frame = img.copy()
			# self.masking()
		except CvBridgeError,e:
			print e

	def callback_gazRGB(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data,"passthrough")
			self.gazFrame = img.copy()
			self.createMini(64*4,48*4)
		except CvBridgeError,e:
			print e

	def callback_record(self,data):
		if self.flagRecord == 0 and data.data == 1:
			now = datetime.now()
			dt_string = now.strftime("%Y_%m_%d_%H_%M_%S")
			fileName = dt_string + "_scara_vs.avi"
			fourcc = cv2.VideoWriter_fourcc('D','I','V','X')
			self.out = cv2.VideoWriter(fileName,fourcc, 10.0, (640,480))
			self.flagRecord = 1

		if self.flagRecord == 1 and data.data == 0:
			self.flagRecord = 0
			self.out.release()

	def createMini(self,w,h):
			self.display = self.gazFrame.copy()
			CamMini = cv2.resize(self.frame,(w,h))
			CamMini[:,0:5] = 0; CamMini[:,-6:-1] = 0;
			CamMini[0:5,:] = 0; CamMini[-6:-1,:] = 0;
			self.display[-1-h:-1,-1-w:-1,:] = CamMini
			if self.flagRecord == 1:
				self.out.write(self.display)

	def masking(self):
		frameHSV = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(frameHSV, (0,50,50), (10,255,255))				# Red
		mask += cv2.inRange(frameHSV, (50,50,50), (70,255,255))			# Green
		mask += cv2.inRange(frameHSV, (110,50,50), (130,255,255))		# Blue
		mask += cv2.inRange(frameHSV, (170,50,50), (180,255,255))		# Red
		self.frameCubes = cv2.bitwise_and(self.frame,self.frame, mask= mask)

if __name__ == '__main__':

	# Unpausing the gazebo physics
	print("Visual Seroving : Waiting for /gazebo/unpause_physics...")
	rospy.wait_for_service('/gazebo/unpause_physics')
	unpause = rospy.ServiceProxy('/gazebo/unpause_physics', EmptySrv)
	unpause()
	print("Visual Seroving : Physics unpaused")
	
	rospy.init_node('SCARA_Cam', anonymous=True)
	sc = scaraCam()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# cv2.imshow('Scara Cam View',sc.frameCubes)
		# cv2.imshow('Gazebo Cam View',sc.gazFrame)
		cv2.imshow('Combined View',sc.display)
		cv2.waitKey(1)
		rate.sleep()