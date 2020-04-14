#!/usr/bin/env python 

'''
# Server(Node) Name : Scara_Server
# Service Name : scara/IK (Service Type Name : IK_srv)
# Service Name : scara/Joint1_Pos_Ref (Service Type Name : PC_srv)
# Service Name : scara/Joint2_Pos_Ref (Service Type Name : PC_srv)
# Service Name : scara/Joint3_Pos_Ref (Service Type Name : PC_srv)
# Service Name : scara/Joint4_Pos_Ref (Service Type Name : PC_srv)
'''

from visual_servoing.srv import *
import rospy
from std_msgs.msg import Float64
import numpy as np
import math as m

np.set_printoptions(suppress=True)

# a,alpha,d,theta
# DH_param = [[0.2 ,0.0 ,0.2    ,q1],
#             [0.2 ,180 ,0.0    ,q2],
#             [0.0 ,0.0 ,0.1+q3 ,0 ],
#             [0.0 ,0.0 ,0.11   ,q4]]

DH_param = [[0.2 ,0.0 ,0.2  ,0],
            [0.2 ,180 ,0.0  ,0],
            [0.0 ,0.0 ,0.1  ,0 ],
            [0.0 ,0.0 ,0.01 ,0]]

def handle_scaraIK(data):
   q3 = 0.2 - (DH_param[2][2] + DH_param[3][2]) - data.pose[2]    #q3
   cos_q2 = (m.pow(data.pose[0],2)+m.pow(data.pose[1],2)-m.pow(DH_param[0][0],2)-m.pow(DH_param[1][0],2))/(2*DH_param[0][0]*DH_param[1][0])
   sin_q2_1 = m.sqrt(1-m.pow(cos_q2,2))
   q2_1 = m.atan2(sin_q2_1,cos_q2)                        #q2 - Solution 1
   q1_1 = m.atan2(data.pose[1],data.pose[0]) - m.atan2(DH_param[1][0]*sin_q2_1,DH_param[0][0]+DH_param[1][0]*cos_q2)    #q1 - Solution 1
   q4_1 = -(data.pose[3]-q1_1-q2_1);                      #q4
   
   # sin_q2_2 = -m.sqrt(1-m.pow(cos_q2,2))
   # q2_2 = m.atan2(sin_q2_2,cos_q2)     #q2 - Solution 2
   # q1_2 = m.atan2(data.pose[1],data.pose[0]) - m.atan2(DH_param[1][0]*sin_q2_2,DH_param[0][0]+DH_param[1][0]*cos_q2)    #q1 - Solution 2

   q = [q1_1,q2_1,q3,q4_1]
   return IK_srvResponse(q)

def handle_J1P(data):
   pub = rospy.Publisher('/scara/joint1_position_controller/command', Float64, queue_size=10)
   pub.publish(data.input)
   return PC_srvResponse("Joint 1 position set")

def handle_J2P(data):
   pub = rospy.Publisher('/scara/joint2_position_controller/command', Float64, queue_size=10)
   pub.publish(data.input)
   return PC_srvResponse("Joint 2 position set")

def handle_J3P(data):
   pub = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=10)
   pub.publish(data.input)
   return PC_srvResponse("Joint 3 position set")

def handle_J4P(data):
   pub = rospy.Publisher('/scara/joint4_position_controller/command', Float64, queue_size=10)
   pub.publish(data.input)
   return PC_srvResponse("Joint 4 position set")

def SCARA_server():
   rospy.init_node('Scara_Server')
   srv_1 = rospy.Service('scara/IK', IK_srv, handle_scaraIK)
   srv_2 = rospy.Service('scara/Joint1_Pos_Ref', PC_srv, handle_J1P)
   srv_3 = rospy.Service('scara/Joint2_Pos_Ref', PC_srv, handle_J2P)
   srv_4 = rospy.Service('scara/Joint3_Pos_Ref', PC_srv, handle_J3P)
   srv_5 = rospy.Service('scara/Joint4_Pos_Ref', PC_srv, handle_J4P)
   print "Scara services ready."
   rospy.spin()
   
# Call function to intiate the service
if __name__ == "__main__":
     SCARA_server()
