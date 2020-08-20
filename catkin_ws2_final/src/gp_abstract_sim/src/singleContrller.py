#!/usr/bin/env python

import math
from math import atan2, pow, sqrt ,pi

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point32, Pose, Twist
from gp_abstract_sim.msg import path_points
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf.transformations import euler_from_quaternion
from planarMovement import *

###########################################################################

currentPath = path_points()
size = 0

dittoS = 0
dittoG = 0

lastX = 0.0
lastY = 0.0
bLastX = 0.0
bLastY = 0.0
lastAngle = 0.0

nextX = 0.0
nextY = 0.0
delta_x = 0.0
delta_y = 0.0
angleToNext = 0.0
distToNext = 0.0

perf_x = []
perf_y = []
perf_x_round = []
perf_y_round = []

###########################################################################


def pathAcquisition(data):
    global currentPath
    global size

    currentPath.x = data.x
    currentPath.y = data.y
    currentPath.yaw = data.yaw
    currentPath.dOne = data.dOne
    currentPath.dTwo = data.dTwo
    size = len(currentPath.x)


def callbackS(data):
    global current_x
    global current_y
    global current_yaw

    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    rot = data.pose.pose.orientation
    (roll, pitch, current_yaw) = euler_from_quaternion(
        [rot.x, rot.y, rot.z, rot.w])


def callbackG(data):
    global gx
    global gy
    global gyaw

    gx = data.pose.pose.position.x
    gy = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, gyaw) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

###########################################################################


def main():
    global dittoS, dittoG, angleToNext, distToNext
    global nextX, nextY, delta_x, delta_y
    global perf_x, perf_y, perf_x_round, perf_y_round
    global lastX, lastY, bLastY, bLastX, lastAngle

    print("Single Ditto Controller!")

    rospy.init_node('singleController',
                    anonymous=True, disable_signals=True)

    rospy.Subscriber("AStarPath", path_points, pathAcquisition)
    rospy.wait_for_message("AStarPath", path_points, timeout=10)
    while size == 0 and not rospy.is_shutdown():
        print("Waiting For Data...")

    dittoS = str(currentPath.dOne)
    dittoG = str(currentPath.dTwo)
    print 'dittoS=' ,dittoS
    print 'dittoG=' ,dittoG

    rospy.Subscriber("ditto" + dittoS + "/odom", Odometry, callbackS)
    rospy.Subscriber("ditto" + dittoG + "/odom", Odometry, callbackG)

    rospy.wait_for_message("ditto" + dittoS + "/odom", Odometry, timeout=10)
    rospy.wait_for_message("ditto" + dittoG + "/odom", Odometry, timeout=10)

    ##############
    # HENA HALEF EL ROBOT EL TANY W AFARMELO, W momken ADELO ZA2A Odam
    lastX = currentPath.x[size - 1]
    lastY = currentPath.y [size - 1]

    bLastX = currentPath.x[size - 2]
    bLastY = currentPath.y [size - 2]

    lastAngle = atan2((lastY - bLastY),(lastX - bLastX))*180/math.pi
    print 'Last Angle =', lastAngle
    rotate(0.125, lastAngle, dittoG)
    forwardFn(0.1, 0.15, dittoG)

    ##############

    while not rospy.is_shutdown():
       
        for i in range(1, size):
            print 'WE ARE IN '
            print i
            nextX = currentPath.x[i]
            nextY = currentPath.y[i]

            perf_x.append(current_x)
            perf_y.append(current_y)

            delta_x = nextX - current_x
            delta_y = nextY - current_y

            distToNext = sqrt((pow(delta_x, 2))  +  (pow(delta_y, 2)))
            angleToNext = atan2(delta_y, delta_x)*180/math.pi
            print 'distToNext',distToNext
            print 'angletoNext',angleToNext 
            print ' '
            # ROTATE
            rotate(0.125,angleToNext,dittoS)
            # FORWARD
            
            forwardFn(distToNext,0.25,dittoS)
            print ' '
            print ' '

    perf_x.append(current_x)
    perf_y.append(current_y)
    
    print(" ")
    print("Goal Reached! Take care now. Bye bye then")

    perf_x_round = [round(num, 4) for num in perf_x]
    perf_y_round = [round(num, 4) for num in perf_y]

    print("After Rounding")
    print(perf_x_round)
    print(" ")
    print(perf_y_round)
    print(" ")

    plt.figure(num="Actual Path Taken")
    plt.grid(True)
    plt.xlabel("X-Position")
    plt.ylabel("Y-Position")
    plt.plot(perf_x_round, perf_y_round, "-b")
    plt.autoscale(enable=True, axis='both')
    plt.show()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()