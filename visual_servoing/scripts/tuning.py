#!/usr/bin/env python 

import rospy, roslib, sys, time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from visual_servoing.srv import *
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)

flag_rec = 0
x_t1 = []
x_t2 = []
y_Ref = []
y_Meas = []
ctr = 0

def callback_JointMeas(data):
   global flag_rec,x_t2,y_Meas
   if flag_rec == 1:
      t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
      # pos = data.position[2]*100
      pos = data.position[3]*180/np.pi
      x_t2.append(t)
      y_Meas.append(pos)
      print ("Time (sec) : " + str(t) + ", Position (cms) :" + str(pos))

def callback_JointRef(data):
   global flag_rec,x_t1,y_Ref
   if flag_rec == 1:
      t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
      # pos = data.set_point*100
      pos = data.set_point*180/np.pi
      x_t1.append(t)
      y_Ref.append(pos)
   
# Call function to intiate the service
if __name__ == "__main__":
   rospy.init_node('SCARA_Tuning', anonymous = False)
   print("\nWaiting for the required services...")
   rospy.wait_for_service('/scara/Joint4_Pos_Ref')
   J3_service = rospy.ServiceProxy('/scara/Joint4_Pos_Ref', PC_srv)
   print("Required services up and running...")
   print("Starting subscriber")
   rospy.Subscriber("/scara/joint_states", JointState, callback_JointMeas)
   rospy.Subscriber('/scara/joint4_position_controller/state', JointControllerState, callback_JointRef)
   print("Joint 3 Limits : [-6 , 8] cms")
   out = J3_service(0)
   flag = 0
   while (flag == 0):
      x = raw_input("Enter the J3 Reference Value (Type \"q\" to exit) : ")
      if x == "q":
         flag = 1
      else:
         #q3 = float(x)/100
         q3 = float(x)*np.pi/180
         ctr += 1
         # Resetting global variables
         x_t1 = []
         x_t2 = []
         y_Ref = []
         y_Meas = []
         # Start recording data here (Record data for 1 second)
         flag_rec = 1
         time.sleep(0.2)
         out = J3_service(q3)
         time.sleep(1.0)
         flag_rec = 0
         # Plotting the graph (Graphs get saved in the home folder)
         plt.clf()
         plt.plot(x_t2, y_Meas,'g',x_t1, y_Ref,'--r')
         plt.ylim(-180, 180)
         plt.xlabel("Time in secs")
         plt.ylabel("Angle in degrees")
         plt.title("Joint 1 Reference vs Measured")
         plt.legend(["Joint 1 Measured","Joint 1 Reference"],loc="best")
         plt.grid()
         plt.show()
         # plt.savefig('Graph_'+str(ctr)+'.png')
