# Visual_Servoing

This project was done as a part of Robot Controls course. Project is still under progress.

Pick objects from a turntable and stacking them using visual servoing. The manipulator used is a 4DoF scara arm. 

Sequence of commands to be executed to start the simulation:
1. roslaunch visual_servoing scara.launch
2. roslaunch visual_servoing turntable.launch
3. rosrun visual_servoing 1_CameraFeed.py
4. rosrun visual_servoing 2_PickPlace.py

For grasping objects package from https://github.com/pal-robotics/gazebo_ros_link_attacher has been implemented. The two Gazebo models are attached with a virtual joint in a generalized grasp hack.
