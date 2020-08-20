#!/usr/bin/python
# -*- coding: utf-8 -*-

from rot_temp import *
import rospy

rospy.init_node('rot_temp', anonymous=True, disable_signals=True)


rotate(0.15,45,"1")
rotate(0.15,0,"1")
rotate(0.15,0,"1")
# # forwardFn(0.4,0.4,1)
# ###
# rotate(0.15,45,2)
# rotate(0.15,0,2)
# rotate(0.15,0,2)
#rotate(0.15,45,2)
#forwardFn(0.1,0.4,2)
###
#rotate(0.15,45,1)
#forwardFn(0.1,0.4,1)


