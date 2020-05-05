#!/usr/bin/env python

import rospy, roslib, sys, cv2, time
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
		self.imgProcRGB = rospy.Subscriber("/scaraCam/color/image_proc",Image,self.callback_imgProcRGB)
		self.GazRGB = rospy.Subscriber("/gazeboCam/gazeboCam/rgb",Image,self.callback_gazRGB)
		self.recording = rospy.Subscriber("/camRecorder", Int32, self.callback_record)
		self.frame = np.zeros((480,640,3), np.uint8)
		self.procFrame = np.ones((480,640,3), np.uint8)*255
		self.gazFrame = np.zeros((480,640,3), np.uint8)
		self.frameCubes = np.zeros((480,640,3), np.uint8)
		self.display = np.zeros((480,640,3), np.uint8)
		self.flagRecord = 0
		self.out = 0
		self.procRecFlag = 0
		self.procImgLastTime = 0

	def callback_imgRGB(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data,"passthrough")
			self.frame = img.copy()
		except CvBridgeError,e:
			print e

	def callback_imgProcRGB(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data,"passthrough")
			self.procFrame = img.copy()
			self.procFrame = self.procFrame[240:,160:480,:]
			self.procImgLastTime = time.time()
			self.procRecFlag = 1
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
			fileName = dt_string + "_scara_vs.mp4"
			fourcc = cv2.VideoWriter_fourcc('D','I','V','X')
			self.out = cv2.VideoWriter(fileName,fourcc, 30.0, (640,480))
			self.flagRecord = 1

		if self.flagRecord == 1 and data.data == 0:
			self.flagRecord = 0
			self.out.release()

	def createMini(self,w,h):
			self.display = self.gazFrame.copy()
			CamMiniD = cv2.resize(self.frame,(w,h))
			CamMiniD[:,:5] = 0; CamMiniD[:,-6:] = 0;
			CamMiniD[:5,:] = 0; CamMiniD[-6:,:] = 0;
			self.display[-h:,-w:,:] = CamMiniD
			if self.procRecFlag == 1 and time.time() - self.procImgLastTime < 2:
				CamMiniU = cv2.resize(self.procFrame,(w,h))
				CamMiniU[:,:5] = 0; CamMiniU[:,-6:] = 0;
				CamMiniU[:5,:] = 0; CamMiniU[-6:,:] = 0;
				self.display[:h,-w:,:] = CamMiniU
			cv2.imshow('Combined View',self.display)
			cv2.waitKey(1)
			if self.flagRecord == 1:
				self.out.write(self.display)

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
		rate.sleep()