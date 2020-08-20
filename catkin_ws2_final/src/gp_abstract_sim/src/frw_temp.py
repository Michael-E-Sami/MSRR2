#!/usr/bin/env python
import math
from math import cos, pow, sin, sqrt
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32

###########################################################################

frw_x = 0.0
frw_y = 0.0
frw_yaw = 0.0
start2 = False

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


def forwardFn(goal_x, goal_y, Omega, dittoNum):
    # rospy.init_node('planarMovement', anonymous=True, disable_signals=True)
    print(" ")
    print("Forwarding")

    global frw_x
    global frw_y
    global frw_yaw
    global start2
    start2 = True
    frw_x = 0.0
    frw_y = 0.0
    frw_yaw = 0.0
    frwDitto = str(dittoNum)

    omega = Float32(Omega)

    rospy.Subscriber("ditto" + frwDitto + "/odom", Odometry, callbackFrw)

    leftFrw = rospy.Publisher(
        "ditto" + frwDitto + "/left_wheel_speed", Float32, queue_size=1000)
    rightFrw = rospy.Publisher(
        "ditto" + frwDitto + "/right_wheel_speed", Float32, queue_size=1000)

    rospy.wait_for_message("ditto" + frwDitto + "/odom", Odometry, timeout=5)

    while start2:
        FrwDiff_x = goal_x - frw_x
        FrwDiff_y = goal_y - frw_y

        eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

        if eucFrw >= 0.05:
            print'Dist. in X to goal.x= ', FrwDiff_x
            print'Dist. in Y to goal.y= ', FrwDiff_y
            print("Remaining Distance: "),
            print(eucFrw)
            print " "

            leftFrw.publish(omega)
            rightFrw.publish(omega)
        else:
            print('Stopped Forwarding')
            print(" ")

            leftFrw.publish(0)
            rightFrw.publish(0)
            start2 = False
