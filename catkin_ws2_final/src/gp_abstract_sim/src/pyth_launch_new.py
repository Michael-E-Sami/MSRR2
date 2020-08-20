#!/usr/bin/python
# -*- coding: utf-8 -*-
import roslaunch
import rospy
import os


directory = os.path.dirname(__file__)
directory = directory[0:len(directory)-4]
ditto_num = str(input("please enter the number of your robot:"))
x = str(input("please enter x:"))
y = str(input("please enter y:"))
cli_args = [directory + "/launch/spawn_sdf_new.launch",'robot_name:=ditto' + ditto_num,'sdf_robot_file:=' + directory + '/models/urdf/modelfinal' + ditto_num +'.sdf', 'x:=' + x,'y:=' + y]
roslaunch_args = cli_args[1:]
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()
rospy.sleep(20)