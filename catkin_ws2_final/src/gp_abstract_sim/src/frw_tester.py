#!/usr/bin/python
# -*- coding: utf-8 -*-

from frw_temp import forwardFn
import rospy

rospy.init_node('frw_tester', anonymous=True, disable_signals=True)

x = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

print("Testing Forward")
print(" ")
for i in range(1, len(x)):
    print("Current Goal: ("),
    print(x[i]),
    print(",")
    print(y[i]),
    print(")")
    print(" ")

    forwardFn(x[i], y[i], 0.2, 2)
