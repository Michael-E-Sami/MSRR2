# MSRR2
#### A simple simulation for a two wheeled differential robot that has the ability to attach with other similar robots and can take many forms each with different functionalities.

### To get started to you need to do the following:
The codes are distributed between two ros workspaces which are catkin_ws2_final and catkin_ws2_final using Python2 and Python3 respectively
after downloading the workspaces you need to make the catkin_ws2_final using the command: **catkin_make_isolated**
<ol type="i">
  <li>roslaunch gp_abstract_sim start_testworld.launch</li>
  <li>roslaunch gp_abstract_sim spawn_sdf.launch robot_name:="your robot name"</li>
  <li>rosrun simple_keys readwrite_keys</li>
  <li>rosrun simple_keys readStringAndCut</li>
</ol>
![](Images/Model.png)

1)	The main body of the robot
2)	The tilting mechanism
3)	The wheels
4)	Front face
5)	Front face magnets (South – North)
6)	The rotating mechanism
7)	ToF sensors
8)	IR sensors
9)	Apriltag (ID: 0)
