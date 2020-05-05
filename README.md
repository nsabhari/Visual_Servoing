# Visual_Servoing

This project was done as a part of Robot Controls course at WPI.<br />

The objective of the project is to pick objects from a turntable and stack them at a destination using visual servoing. The manipulator used is a 4 DoF scara arm. This was simulated using ROS and Gazebo.<br />

Sequence of commands to be executed to start the simulation:<br />
- roslaunch visual_servoing scara.launch
  - Launches gazebo with scara manipulator<br />
- roslaunch visual_servoing turntable.launch
  - Spaws the tunrtable and objects in gazebo<br />
- rosrun visual_servoing 1_CameraFeed.py
  - Combines the various camera feeds and processed feeds into a single feed (Also used for video recording)<br />

If you want to try the pick and place with static turntable then run (No visual servoing)<br />
- rosrun visual_servoing 2_PickPlace_Static.py<br />
  
If you want to try the pick and place with rotating turntable then run (With visual servoing)<br />
- rosrun visual_servoing 3_PickPlace_VS.py<br />

services.py script : Consists of services used by other scripts to control the manipulator<br />
tuning.py script   : Used to assist in the tuning process of PD gains for the manipulator joints<br />

For grasping objects package from https://github.com/pal-robotics/gazebo_ros_link_attacher has been implemented. The two Gazebo models are attached with a virtual joint in a generalized grasp hack.<br />
