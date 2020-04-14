#!/usr/bin/env python

import rospy, roslib, sys, time
import numpy as np
from std_msgs.msg import Int32
from std_srvs.srv import Empty as EmptySrv
from visual_servoing.srv import *
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

np.set_printoptions(suppress=True)

modelParent = ["scara"]
linkParent  = ["scara_link4"]
modelChild  = ["cube_red","cube_green","cube_blue"]
linkChild   = ["cube_red","cube_green","cube_blue"]
pickPos     = [[0.2,-0.25,0.0275,0],
							 [0.1567,-0.175,0.0275,0],
							 [0.2433,-0.175,0.0275,0]]
placePos	  = [[0.2,0.2,0.028,0],
							 [0.2,0.2,0.0485,0],
							 [0.2,0.2,0.069,0]]
homePos			= [0.4,0,0.05,0];

def SetPoint(pos,clearFlag,waitTime):
	res = IKSrv(pos).q;
	J1Srv(res[0]);
	J2Srv(res[1]);
	if clearFlag == 0: J3Srv(res[2])
	else:							 J3Srv(-0.5)
	J4Srv(res[3]); time.sleep(1)
	time.sleep(waitTime)

if __name__ == '__main__':
	rospy.init_node('SCARA_VS')

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
	
	pubRecord = rospy.Publisher('/camRecorder', Int32, queue_size=1)

	unpause()			# Unpausing the gazebo physics
	time.sleep(1)

	pubRecord.publish(1); time.sleep(2)

	SetPoint(homePos,0,1)
	
	for i in range(3):
		SetPoint(pickPos[i],1,1.50)		# Go to pick location

		SetPoint(pickPos[i],0,0.75)		# Move down
		res = AttSrv(modelParent[0],linkParent[0],modelChild[i],linkChild[i]); time.sleep(0.1)	# Grasp the object
		SetPoint(pickPos[i],1,0.75)		# Move up
		
		SetPoint(placePos[i],1,1.50)	# Go to place location

		SetPoint(placePos[i],0,0.75)	# Move down
		res = DetSrv(modelParent[0],linkParent[0],modelChild[i],linkChild[i]); time.sleep(0.1)	# Detach the object
		SetPoint(placePos[i],1,0.75)	# Move up

	SetPoint(homePos,0,1)
	time.sleep(2)
	pubRecord.publish(0) 