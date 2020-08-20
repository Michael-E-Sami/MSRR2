#!/usr/bin/env python

import math
import os.path
from math import atan2, cos, pi, pow, sin, sqrt

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Point32, Pose, Twist
from gp_abstract_sim.msg import path_points
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf.transformations import euler_from_quaternion

###########################################################################

whichDitto = [0, 0]

currentX = [0, 0]
currentY = [0, 0]
currentYawDeg = [0, 0]
currentYawRad = [0, 0]


###########################################################################


def pathAcquisition(data):
    global currentPath
    global size
    currentPath = path_points()

    currentPath.x = data.x
    currentPath.y = data.y
    currentPath.yaw = data.yaw
    currentPath.dOne = data.dOne
    currentPath.dTwo = data.dTwo
    size = len(currentPath.x)


def movingDittoCB(data):
    global currentX
    global currentY
    global currentYawRad, currentYawDeg

    currentX[0] = data.pose.pose.position.x
    currentY[0] = data.pose.pose.position.y
    rot = data.pose.pose.orientation
    (roll, pitch, currentYawRad[0]) = euler_from_quaternion(
        [rot.x, rot.y, rot.z, rot.w])

    currentYawDeg[0] = currentYawRad[0]*(180/pi)

    if currentYawDeg[0] < 0:
        currentYawDeg[0] += 360


def stableDittoCB(data):
    global currentX
    global currentY
    global currentYawRad, currentYawDeg

    currentX[1] = data.pose.pose.position.x
    currentY[1] = data.pose.pose.position.y
    rot = data.pose.pose.orientation
    (roll, pitch, currentYawRad[1]) = euler_from_quaternion(
        [rot.x, rot.y, rot.z, rot.w])

    currentYawDeg[1] = currentYawRad[1]*(180/pi)

    if currentYawDeg[1] < 0:
        currentYawDeg[1] += 360

###########################################################################


# def forwardFnDist(Distance, Omega, dittoNum, leftFrw, rightFrw):
#     start2 = True
#     FrwDiff_x = 0
#     FrwDiff_y = 0
#     eucFrw = 0
#     indexFrw = 69  # Error Num

#     print("Forwarding")

#     for n in range(0,len(whichDitto)):
#         if whichDitto[n] == dittoNum:
#             indexFrw = n

#     distance = float(Distance)
#     goal = Point()
#     goal.x= distance *cos(currentYawRad[indexFrw]) + currentX[indexFrw]
#     goal.y= distance *sin(currentYawRad[indexFrw])  + currentY[indexFrw]

#     while start2:
#         FrwDiff_x = goal.x - currentX[indexFrw]
#         FrwDiff_y = goal.y - currentY[indexFrw]

#         eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

#         if eucFrw >= 0.05:
#             print'Dist. in X to goal.x= ', FrwDiff_x
#             print'Dist. in Y to goal.y= ', FrwDiff_y
#             print("Remaining Distance: "),
#             print(eucFrw)
#             print " "

#             leftFrw.publish(Omega)
#             rightFrw.publish(Omega)

#         else:
#             print('Stopped Forwarding')
#             print(" ")

#             leftFrw.publish(0)
#             rightFrw.publish(0)
#             start2 = False



###########################################################################


def forwardFn(goal_x, goal_y, Omega, dittoNum, leftFrw, rightFrw):
    global start2, currentX, currentY, currentYawRad, currentYawDeg, whichDitto
    start2 = True
    FrwDiff_x = 0
    FrwDiff_y = 0
    eucFrw = 0
    indexFrw = 69  # Error Num

    print("Forwarding")

    for n in range(0,len(whichDitto)):
        if whichDitto[n] == dittoNum:
            indexFrw = n

    while start2:
        FrwDiff_x = goal_x - currentX[indexFrw]
        FrwDiff_y = goal_y - currentY[indexFrw]

        eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

        if eucFrw >= 0.025:
            print'Dist. in X to goal.x= ', FrwDiff_x
            print'Dist. in Y to goal.y= ', FrwDiff_y
            print("Remaining Distance: "),
            print(eucFrw)
            print " "

            leftFrw.publish(Omega)
            rightFrw.publish(Omega)

        else:
            print('Stopped Forwarding')
            print(" ")

            leftFrw.publish(0)
            rightFrw.publish(0)
            start2 = False

###########################################################################


def rotate(angle, Omega, dittoNum, left_pub, right_pub):
    global start, currentX, currentY, currentYawRad, currentYawDeg, whichDitto
    start = True
    indexRot = 69
    yaw_goal = 0

    print ('Rotating...')

    for n in range(0,len(whichDitto)):
        if whichDitto[n] == dittoNum:
            indexRot = n

    save_path = os.path.dirname(__file__)
    name_of_file = "last_yaw_goal"
    completeName = os.path.join(save_path, name_of_file+".txt")
    file1 = open(completeName, "r")

    # Note that position == n-1 for the nth line.
    for position, line in enumerate(file1):
        # If line(position) desired is first line, then position == 0
        if position == dittoNum:
            dittoTXT = line

    dittoTXT = float(dittoTXT)
    file1.close()

    yaw_goal = angle + dittoTXT
    if yaw_goal < 0:
        yaw_goal = yaw_goal + 359.99
    elif yaw_goal > 359.99:
        yaw_goal = yaw_goal - 359.99

    file1 = open(completeName, "r")
    list_of_lines = file1.readlines()
    list_of_lines[int(dittoNum)] = (str(yaw_goal) + "\n")

    file1 = open(completeName, "w")
    file1.writelines(list_of_lines)
    file1.close()

    while start:
        #print('yaw goal - current yaw = ', yaw_goal - currentYawDeg[indexRot])
        if abs(yaw_goal - currentYawDeg[indexRot]) > 0.1:

            if (angle > 0):
                #print 'Angle > 0  : Cuurent Yaw Deg = ', currentYawDeg[indexRot]
                left_pub.publish(-Omega)
                right_pub.publish(Omega)
            elif angle < 0:
                #print 'Angle < 0  : Cuurent Yaw Deg = ', currentYawDeg[indexRot]
                left_pub.publish(Omega)
                right_pub.publish(-Omega)
            elif angle == 0:
                Omega = 0.025
                if yaw_goal - currentYawDeg[indexRot] > 0:
                    #print 'Right : Angle = 0, Current Yaw = ', currentYawDeg[indexRot]
                    left_pub.publish(-Omega)
                    right_pub.publish(Omega)
                else:
                    #print 'Left : Angle = 0, Current Yaw = ', currentYawDeg[indexRot]
                    left_pub.publish(Omega)
                    right_pub.publish(-Omega)

        else:
            left_pub.publish(0)
            right_pub.publish(0)
            print 'Finished Rotating, Current Yaw Deg = ', currentYawDeg[indexRot]
            print 'Finished Rotating, Current Yaw Rad = ', currentYawRad[indexRot]
            start = False

###########################################################################


def main():
    global movingDitto, stableDitto
    perf_x = []
    perf_y = []
    perf_x_round = []
    perf_y_round = []
    angleToNext_old = 0
    print("Single Ditto Controller!")

    rospy.init_node('singleController', anonymous=True, disable_signals=True)

    rospy.Subscriber("AStarPath", path_points, pathAcquisition)
    rospy.wait_for_message("AStarPath", path_points, timeout=10)

    movingDitto = str(currentPath.dOne)
    stableDitto = str(currentPath.dTwo)
    whichDitto[0] = int(movingDitto)  # Raqam El movingDitto Byet7at Fe Awel 5ana
    whichDitto[1] = int(stableDitto)  # Raqam El stableDitto Byet7at Fe Tani 5ana

    rospy.Subscriber("ditto" + movingDitto + "/odom", Odometry, movingDittoCB)
    rospy.Subscriber("ditto" + stableDitto + "/odom", Odometry, stableDittoCB)

    rospy.wait_for_message("ditto" + movingDitto +
                           "/odom", Odometry, timeout=10)
    rospy.wait_for_message("ditto" + stableDitto +
                           "/odom", Odometry, timeout=10)

    movingLeftWheel = rospy.Publisher(
        "ditto" + movingDitto + "/left_wheel_speed", Float32, queue_size=1000)
    movingRightWheel = rospy.Publisher(
        "ditto" + movingDitto + "/right_wheel_speed", Float32, queue_size=1000)

    stableLeftWheel = rospy.Publisher(
        "ditto" + stableDitto + "/left_wheel_speed", Float32, queue_size=1000)
    stableRightWheel = rospy.Publisher(
        "ditto" + stableDitto + "/right_wheel_speed", Float32, queue_size=1000)

    ##############
    # HENA HALEF EL ROBOT EL TANY W AFARMELO, W momken ADELO ZA2A Odam
    lastX = currentPath.x[size - 1]
    lastY = currentPath.y[size - 1]

    bLastX = currentPath.x[size - 2]
    bLastY = currentPath.y[size - 2]

    lastAngle = atan2((lastY - bLastY), (lastX - bLastX))*180/math.pi
    lastAngleRad = atan2((lastY - bLastY), (lastX - bLastX))

    # Howa Hena Lazem Ab3at el Difference Wala Last Angle Bs? I Guess Diff
    rotate(lastAngle, 0.15, int(stableDitto),
           stableLeftWheel, stableRightWheel)
    rotate(0, 0.15, int(stableDitto), stableLeftWheel, stableRightWheel)
    # rotate(0, 0.15, int(stableDitto), stableLeftWheel, stableRightWheel)

    tempX1 = (0.25*cos(lastAngleRad)) + currentX[1]
    tempY1 = (0.25*sin(lastAngleRad)) + currentY[1]

    forwardFn(tempX1, tempY1, 0.15, int(stableDitto),
              stableLeftWheel, stableRightWheel)

    ##############

    while not rospy.is_shutdown():
        for i in range(1, size):

            nextX = currentPath.x[i]
            nextY = currentPath.y[i]

            perf_x.append(currentX[0])
            perf_y.append(currentY[0])

            delta_x = nextX - currentX[0]
            delta_y = nextY - currentY[0]

            distToNext = sqrt((pow(delta_x, 2)) + (pow(delta_y, 2)))

            angleToNext = atan2(delta_y, delta_x)*180/math.pi
            angleToNext_diff = angleToNext - angleToNext_old
            angleToNext_old = angleToNext
            print 'current path.x[i]= ', currentPath.x[i]
            print 'currenpt path.y[i]= ', currentPath.y[i]

            print 'Moving Ditto Actual Yaw (Deg)= ', currentYawDeg[0]
            print 'Angle to Next Point= ', angleToNext
            print 'Old Angle to Next= ', angleToNext_old
            print 'Diff. Between Them = ', angleToNext_diff
            print ' '
            print 'i=', i
            print 'current path.x[i]= ', currentPath.x[i]
            print 'currenpt path.y[i]= ', currentPath.y[i]
            print ' '
            print 'current x[0]= ', currentX[0]
            print 'current y[0]= ', currentY[0]
            if i== size-1:
                rospy.sleep(10)
               
            # ROTATE
            rotate(angleToNext_diff, 0.15, int(movingDitto),
                   movingLeftWheel, movingRightWheel)
            rotate(0, 0.15, int(movingDitto),
                   movingLeftWheel, movingRightWheel)
            rotate(0, 0.15, int(movingDitto),
                    movingLeftWheel, movingRightWheel)
            print 'current yaw = ', currentYawRad[0]
            # FORWARD
            tempX0 = (distToNext*cos(currentYawRad[0])) + currentX[0]
            tempY0 = (distToNext*sin(currentYawRad[0])) + currentY[0]
            print 'tempX0 =' , tempX0
            print 'tempY0 =' , tempY0

            forwardFn(tempX0, tempY0, 0.2, int(movingDitto),
                      movingLeftWheel, movingRightWheel)
            # forwardFn(distToNext, 0.25, int(movingDitto))

        perf_x.append(currentX[0])
        perf_y.append(currentY[0])

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
        angodiff=atan2((currentY[1]-currentY[0]),(currentX[1]-currentX[0]))
        print"ANGIO DIFF:: ",angodiff
        rotate(angodiff, 0.15, int(movingDitto),movingLeftWheel, movingRightWheel)
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    main()
