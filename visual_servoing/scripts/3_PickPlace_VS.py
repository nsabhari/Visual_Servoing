#!/usr/bin/env python

import rospy, roslib, sys, cv2, time
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from visual_servoing.srv import *
from std_srvs.srv import Empty as EmptySrv
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime 

modelParent = ["scara"]
linkParent  = ["scara_link4"]
modelChild  = ["cube_red","cube_green","cube_blue"]
linkChild   = ["cube_red","cube_green","cube_blue"]
pickPos     = [0.16,-0.2,0.8,0]
placePos	  = [[0.2,0.2,0.028,0],
							 [0.2,0.2,0.0485,0],
							 [0.2,0.2,0.069,0]]
homePos			= [0.4,0,0.05,0];

class scaraCam:
	def __init__(self):
		self.bridge = CvBridge()
		self.imgRGB = rospy.Subscriber("/scaraCam/color/image_raw",Image,self.callback_imgRGB)
		self.pubProcImg = rospy.Publisher('/scaraCam/color/image_proc', Image, queue_size=10)
		self.frame = np.zeros((480,640,3), np.uint8)
		self.frameCubes = np.zeros((480,640,3), np.uint8)
		self.font = cv2.FONT_HERSHEY_SIMPLEX
		self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))

	def callback_imgRGB(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data,"passthrough")
			self.frame = img.copy()
		except CvBridgeError,e:
			print e

	def masking(self,color):
		self.frameCubes = self.frame.copy()
		frameHSV = cv2.cvtColor(self.frameCubes, cv2.COLOR_BGR2HSV)
		if color == "red":
			mask = cv2.inRange(frameHSV, (0,50,240), (10,255,255))		
			mask += cv2.inRange(frameHSV, (170,50,240), (180,255,255))
		elif color == "green":
			mask = cv2.inRange(frameHSV, (50,50,240), (70,255,255))
		else:
			mask = cv2.inRange(frameHSV, (110,50,240), (130,255,255))

		# mask = cv2.erode(mask,self.kernel,iterations = 1)
		# mask = cv2.dilate(mask,self.kernel,iterations = 1)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

		_,contours,_ = cv2.findContours(mask, 1, 2)
		max_area = 0; c = 0; i=0
		while i < len(contours):
			area = cv2.contourArea(contours[i])
			if max_area < area:
				max_area = area
				c = i
			i += 1

		if max_area >= 4000:
			rect = cv2.minAreaRect(contours[c])
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			self.frameCubes = cv2.drawContours(self.frameCubes,[box],0,(0,0,0),5)
			# cv2.putText(self.frameCubes,str(np.round(rect[0],2)),(20,20), self.font, .5,(0,0,0),2,cv2.LINE_AA)
			# cv2.putText(self.frameCubes,str(np.round(rect[2],2)),(20,40), self.font, .5,(0,0,0),2,cv2.LINE_AA)
			cv2.circle(self.frameCubes,(320,395), 7, (255,255,255), -1)
			cv2.circle(self.frameCubes,(int(round(rect[0][0],0)),int(round(rect[0][1],0))), 5, (0,0,0), -1)
			self.pubProcImg.publish(self.bridge.cv2_to_imgmsg(self.frameCubes,"bgr8"))
			# cv2.imshow('Scara Cam View',sc.frameCubes)
			# cv2.waitKey(1)
		
		if max_area >= 4000:
			return([rect[0][0],rect[0][1],rect[2],max_area])
		else:
			return([0,0,0,0,0])

q = [0,0,0,0]
pos = [0,0,0,0]
def callback_JointMeas(data):
	 global q,pos
	 q = data.position
	 pos[0] = 0.2*np.cos(q[0])+0.2*np.cos(q[0]+q[1])
	 pos[1] = 0.2*np.sin(q[0])+0.2*np.sin(q[0]+q[1])
	 pos[2] = 0.2-q[2]-0.1-0.01
	 
def RotZ(theta):
	return([[np.cos(theta),-np.sin(theta)],
					[np.sin(theta), np.cos(theta)]])

def SetPoint(pos,clearFlag,waitTime):
	res = IKSrv(pos).q;
	J1Srv(res[0]);
	J2Srv(res[1]);
	if clearFlag == 0: 		J3Srv(res[2])
	elif clearFlag == 1:  J3Srv(-0.5)
	else:									J3Srv(res[2]-0.01)
	J4Srv(res[3]);
	time.sleep(waitTime)

if __name__ == '__main__':
	rospy.init_node('SCARA_CamProcess', anonymous=True)

	# Unpausing the gazebo physics
	print("Visual Seroving : Waiting for required services...")
	rospy.wait_for_service('/gazebo/unpause_physics')
	rospy.wait_for_service('/scara/IK')
	rospy.wait_for_service('/scara/Joint1_Pos_Ref')
	rospy.wait_for_service('/scara/Joint2_Pos_Ref')
	rospy.wait_for_service('/scara/Joint3_Pos_Ref')
	rospy.wait_for_service('/scara/Joint4_Pos_Ref')
	rospy.wait_for_service('/link_attacher_node/attach')
	print("Visual Seroving : All services ready...")
	
	unpause = rospy.ServiceProxy('/gazebo/unpause_physics', EmptySrv)
	IKSrv 	= rospy.ServiceProxy('/scara/IK', IK_srv)
	J1Srv 	= rospy.ServiceProxy('/scara/Joint1_Pos_Ref', PC_srv)
	J2Srv 	= rospy.ServiceProxy('/scara/Joint2_Pos_Ref', PC_srv)
	J3Srv 	= rospy.ServiceProxy('/scara/Joint3_Pos_Ref', PC_srv)
	J4Srv 	= rospy.ServiceProxy('/scara/Joint4_Pos_Ref', PC_srv)
	AttSrv  = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
	DetSrv  = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

	pubTT = rospy.Publisher("/turntable/tta_velocity_controller/command",Float64, queue_size=10)
	pubRecord = rospy.Publisher('/camRecorder', Int32, queue_size=1)
	rospy.Subscriber("/scara/joint_states", JointState, callback_JointMeas)

	unpause()			# Unpausing the gazebo physics
	time.sleep(1)

	pubTT.publish(1)
	frameSize = [640,480]
	desired = [320,395]
	Kp = 0.6
	sc = scaraCam()

	# pubRecord.publish(1); time.sleep(2)

	SetPoint(homePos,0,2)

	for i in range(3):
		SetPoint(pickPos,1,2)		# Go to pick location

		flagDown = 0
		start = time.time()
		# Track the cube till its grasped	
		while not rospy.is_shutdown():
			if i == 0:
				res = sc.masking("red")
			elif i == 1:
				res = sc.masking("green")
			else:
				res = sc.masking("blue")

			if res[3] != 0:
				error = np.array([res[1]-desired[1],res[0]-desired[0]])
				error = np.dot(RotZ(q[0]+q[1]),error)
				dPos = -Kp*error*(0.03/120)
				newPos = np.array(pos[0:2])+dPos
				# print(np.degrees(np.unwrap([np.radians(res[2])+q[0]+q[1]])))
				newPos = np.concatenate((newPos,[0.0275,-np.radians(res[2])+q[0]+q[1]]),axis = None)
				if np.linalg.norm(dPos)*1000 < 1 or flagDown == 1:
					SetPoint(newPos,0,0)
					flagDown = 1
					if time.time()-start > 5 and np.linalg.norm(dPos)*1000 < 0.5 and res[3] > 7000 and abs(newPos[2]-pos[2])*1000<1:
						AttSrv(modelParent[0],linkParent[0],modelChild[i],linkChild[i]); time.sleep(0)	# Grasp the object
						SetPoint(newPos,2,0.25)	# Clear table after grasping
						# print("Cube "+str(i+1)+" Grasped")
						SetPoint(newPos,1,0.75)	# Retract after grasping
						break
				else:
					SetPoint(newPos,1,0)

		SetPoint(placePos[i],1,1.50)	# Go to place location

		# Changing turntable directions
		if i==0:
			pubTT.publish(-1)
		elif i==1:
			pubTT.publish(1)

		SetPoint(placePos[i],0,0.75)	# Move down
		res = DetSrv(modelParent[0],linkParent[0],modelChild[i],linkChild[i]); time.sleep(1.0)	# Detach the object
		SetPoint(placePos[i],1,0.75)	# Move up

	SetPoint(homePos,0,1)
	# time.sleep(2); pubRecord.publish(0) 