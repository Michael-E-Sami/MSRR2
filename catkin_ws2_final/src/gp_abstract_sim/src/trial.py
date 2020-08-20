#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, floor
import math
from gazebo_msgs.msg import ModelStates 
from math import *
from sensor_msgs.msg import Range


#def callback(msg):
#	print msg
#def callback1(msg1):
#	print msg1
if __name__ == "__main__":
	
	rospy.init_node('try')
	pub =rospy.Publisher("cmd_vel", Twist, queue_size=10)
	#rospy.Subscriber("body_ir", Range, callback1)
	rate = rospy.Rate(2)
	#while not rospy.is_shutdown():
	msg = Twist()
	pub.publish(msg)
	rospy.spin()
		#rate.sleep

	




