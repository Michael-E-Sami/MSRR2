#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import cos, pow, sin, sqrt
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

# from planarMovement import *

# Ana 3ayez a-Define Function
# Bta5od Distance
# W esm el Ditto / El publishers bto3o


###########################################################################

frw_x = 0.0
frw_y = 0.0
frw_yaw = 0.0

###########################################################################


def callbackFrw(data):
    global frw_x
    global frw_y
    global frw_yaw

    frw_x = data.pose.pose.position.x
    frw_y = data.pose.pose.position.y
    rotFrw = data.pose.pose.orientation
    (rotFrwroll, rotFrwpitch, frw_yaw) = euler_from_quaternion(
        [rotFrw.x, rotFrw.y, rotFrw.z, rotFrw.w])

###########################################################################


def forwardFn(Distance, Omega, dittoNum):
    # rospy.init_node('planarMovement', anonymous=True, disable_signals=True)
    print(" ")
    print("Forwarding")

    frwDitto = str(dittoNum)

    omega = Float32(Omega)

    rospy.Subscriber("ditto" + frwDitto + "/odom", Odometry, callbackFrw)

    leftFrw = rospy.Publisher(
        "ditto" + frwDitto + "/left_wheel_speed", Float32, queue_size=1000)
    rightFrw = rospy.Publisher(
        "ditto" + frwDitto + "/right_wheel_speed", Float32, queue_size=1000)

    rospy.wait_for_message("ditto" + frwDitto + "/odom", Odometry, timeout=5)

    distance = float(Distance)
    goal = Point()
    goal.x= distance *sin(frw_yaw) + frw_x
    goal.y= distance *cos(frw_yaw)  + frw_y

    while not rospy.is_shutdown():
        FrwDiff_x = goal.x - frw_x
        FrwDiff_y = goal.y - frw_y

        eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

        print'Dist. in X to goal.x= ', FrwDiff_x
        print'Dist. in Y to goal.y= ', FrwDiff_y
        print'euc= ', eucFrw

        while eucFrw > 0.05 and not rospy.is_shutdown():
            FrwDiff_x = goal.x - frw_x
            FrwDiff_y = goal.y - frw_y

            eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

            print("Inside While Loop")
            print(frw_x)

            leftFrw.publish(omega)
            rightFrw.publish(omega)

        print("Left While Loop")

        leftFrw.publish(0)
        rightFrw.publish(0)
        break
