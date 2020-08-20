#!/usr/bin/env python

import math
import os.path
from math import atan2, cos, pi, pow, sin, sqrt

import rospy
from geometry_msgs.msg import Point
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

pub = {}

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


def forwardFn(goal_x, goal_y, Omega, dittoNum):
    global start2, currentX, currentY, currentYawRad, currentYawDeg, whichDitto
    start2 = True
    FrwDiff_x = 0
    FrwDiff_y = 0
    eucFrw = 0
    indexFrw = 69  # Error Num

    print("Forwarding")

    # Mafrood Neb3at dittoNum As a String

    for n in range(0, len(whichDitto)):
        print ' n = ', n
        print 'which ditto', whichDitto[n]
        if whichDitto[n] == int(dittoNum[0]):
            indexFrw = n

    print 'indexFrw', indexFrw
    print ' '
    print 'dittoNum', dittoNum[0]
    print 'which ditto 1 ', whichDitto[1]

    while start2:
        FrwDiff_x = goal_x - currentX[indexFrw]
        FrwDiff_y = goal_y - currentY[indexFrw]

        eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

        if eucFrw >= 0.01:
            # print'Dist. in X to goal.x= ', FrwDiff_x
            # print'Dist. in Y to goal.y= ', FrwDiff_y
            # print("Remaining Distance: "),
            # print(eucFrw)
            # print " "

            for i in str(dittoNum):
                pub["ditto" + i + "/left_wheel_speed"].publish(Omega)
                pub["ditto" + i + "/right_wheel_speed"].publish(Omega)

        else:
            print('Stopped Forwarding')
            print(" ")

            for i in str(dittoNum):
                pub["ditto" + i + "/left_wheel_speed"].publish(0)
                pub["ditto" + i + "/right_wheel_speed"].publish(0)

            start2 = False


def rotate(angle, Omega, dittoNum):
    global start, currentX, currentY, currentYawRad, currentYawDeg, whichDitto
    start = True
    indexRot = 69
    yaw_goal = 0

    print ('Rotating...')

    for n in range(0, len(whichDitto)):
        if whichDitto[n] == int(dittoNum[0]):
            indexRot = n

    save_path = os.path.dirname(__file__)
    name_of_file = "last_yaw_goal"
    completeName = os.path.join(save_path, name_of_file+".txt")
    file1 = open(completeName, "r")

    # Note that position == n-1 for the nth line.
    for position, line in enumerate(file1):
        # If line(position) desired is first line, then position == 0
        if position == int(dittoNum[0]):
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
    for i in range(0, len(dittoNum)):
        list_of_lines[int(dittoNum[i])] = (str(yaw_goal))+ "\n"

    file1 = open(completeName, "w")
    file1.writelines(list_of_lines)
    file1.close()

    while start:
        #print('yaw goal - current yaw = ', yaw_goal - currentYawDeg[indexRot])
        if abs(yaw_goal - currentYawDeg[indexRot]) > 0.1:

            if (angle > 0):
                #print 'Angle > 0  : Cuurent Yaw Deg = ', currentYawDeg[indexRot]
                for i in str(dittoNum):
                    pub["ditto" + i + "/left_wheel_speed"].publish(-Omega)
                    pub["ditto" + i + "/right_wheel_speed"].publish(Omega)
            elif angle < 0:
                #print 'Angle < 0  : Cuurent Yaw Deg = ', currentYawDeg[indexRot]
                for i in str(dittoNum):
                    pub["ditto" + i + "/left_wheel_speed"].publish(Omega)
                    pub["ditto" + i + "/right_wheel_speed"].publish(-Omega)
            elif angle == 0:
                Omega = 0.025
                if yaw_goal - currentYawDeg[indexRot] > 0:
                    #print 'Right : Angle = 0, Current Yaw = ', currentYawDeg[indexRot]
                    for i in str(dittoNum):
                        pub["ditto" + i + "/left_wheel_speed"].publish(-Omega)
                        pub["ditto" + i + "/right_wheel_speed"].publish(Omega)
                else:
                    #print 'Left : Angle = 0, Current Yaw = ', currentYawDeg[indexRot]
                    for i in str(dittoNum):
                        pub["ditto" + i + "/left_wheel_speed"].publish(Omega)
                        pub["ditto" + i + "/right_wheel_speed"].publish(-Omega)

        else:
            for i in str(dittoNum):
                pub["ditto" + i + "/left_wheel_speed"].publish(0)
                pub["ditto" + i + "/right_wheel_speed"].publish(0)

            print 'Finished Rotating, Current Yaw Deg = ', currentYawDeg[indexRot]
            print 'Finished Rotating, Current Yaw Rad = ', currentYawRad[indexRot]
            start = False


###########################################################################


def main():
    global movingDitto, stableDitto, multiDitto
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
    # Raqam El movingDitto Byet7at Fe Awel 5ana
    whichDitto[0] = int(movingDitto)

    rospy.Subscriber("ditto" + movingDitto + "/odom", Odometry, movingDittoCB)

    # Raqam El stableDitto Byet7at Fe Tani 5ana
    print 'stable ditto', stableDitto
    whichDitto[1] = int(stableDitto[0])
    print 'type stable ditto 0 =', type(stableDitto[0])
    print 'type stable ditto 0 =', type(whichDitto[1])

    rospy.Subscriber(
        "ditto" + stableDitto[0] + "/odom", Odometry, stableDittoCB)

    stableDittoSize = len(stableDitto)

    if stableDittoSize > 1:
        multiDitto = True
    elif len(stableDitto) == 1:
        multiDitto = False

    rospy.wait_for_message("ditto" + movingDitto +
                           "/odom", Odometry, timeout=10)
    rospy.wait_for_message("ditto" + stableDitto[0] +
                           "/odom", Odometry, timeout=10)

    pub["ditto" + movingDitto + "/left_wheel_speed"] = rospy.Publisher(
        "ditto" + movingDitto + "/left_wheel_speed", Float32, queue_size=1000)
    pub["ditto" + movingDitto + "/right_wheel_speed"] = rospy.Publisher(
        "ditto" + movingDitto + "/right_wheel_speed", Float32, queue_size=1000)

    for i in str(stableDitto):
        pub["ditto"+i+"/left_wheel_speed"] = rospy.Publisher(
            "ditto" + i + "/left_wheel_speed", Float32, queue_size=1000)
        pub["ditto"+i+"/right_wheel_speed"] = rospy.Publisher(
            "ditto" + i + "/right_wheel_speed", Float32, queue_size=1000)

    ##############
    # HENA HALEF EL ROBOT EL TANY W AFARMELO, W momken ADELO ZA2A Odam
    lastX = currentPath.x[size - 1]
    lastY = currentPath.y[size - 1]

    bLastX = currentPath.x[size - 2]
    bLastY = currentPath.y[size - 2]

    lastAngle = atan2((lastY - bLastY), (lastX - bLastX))*180/math.pi
    lastAngleRad = atan2((lastY - bLastY), (lastX - bLastX))

    # Howa Hena Lazem Ab3at el Difference
    rotate(lastAngle, 0.1, str(stableDitto[0]))
    rospy.sleep(0.5)
    rotate(0, 0.15, str(stableDitto[0]))
    rospy.sleep(0.5)
    rotate(0, 0.15, str(stableDitto[0]))

    tempX1 = (0.16*cos(lastAngleRad)) + currentX[1]
    tempY1 = (0.16*sin(lastAngleRad)) + currentY[1]

    forwardFn(tempX1, tempY1, 0.15, str(stableDitto[0]))

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

            print 'current path.x[i] = ', currentPath.x[i]
            print 'currenpt path.y[i] = ', currentPath.y[i]
            print ' '
            print 'Moving Ditto Actual Yaw (Deg) = ', currentYawDeg[0]
            print 'Angle to Next Point = ', angleToNext
            print 'Old Angle to Next = ', angleToNext_old
            print 'Diff. Between Them = ', angleToNext_diff
            print ' '
            print 'For Loop : Index = ', i
            print 'current path.x[i] = ', currentPath.x[i]
            print 'currenpt path.y[i] = ', currentPath.y[i]
            print ' '
            print 'current x[0] = ', currentX[0]
            print 'current y[0] = ', currentY[0]
            # if i== size-1:
            #     rospy.sleep(10)

            # ROTATE
            rotate(angleToNext_diff, 0.15, str(movingDitto[0]))
            rospy.sleep(0.5)
            rotate(0, 0.15, str(movingDitto[0]))
            rospy.sleep(0.5)
            rotate(0, 0.15, str(movingDitto[0]))

            print 'Current Yaw Rad [0] = ', currentYawRad[0]

            # FORWARD
            tempX0 = (distToNext*cos(currentYawRad[0])) + currentX[0]
            tempY0 = (distToNext*sin(currentYawRad[0])) + currentY[0]
            print 'tempX0 =', tempX0
            print 'tempY0 =', tempY0

            forwardFn(tempX0, tempY0, 0.2, str(movingDitto))
            # forwardFn(distToNext, 0.25, int(movingDitto))

        perf_x.append(currentX[0])
        perf_y.append(currentY[0])

        print(" ")
        print("--> Goal Reached! <--")

        perf_x_round = [round(num, 4) for num in perf_x]
        perf_y_round = [round(num, 4) for num in perf_y]

        print("Actual Path Points Achieved: ")
        print(perf_x_round)
        print(" ")
        print(perf_y_round)
        print(" ")

        j = lastAngle-currentYawDeg[0]
        print 'Last Angle - Current Yaw Deg [0] = ', j

        rotate(j, 0.1, str(movingDitto[0]))
        rospy.sleep(0.5)
        rotate(0, 0.1, str(movingDitto[0]))
        rospy.sleep(0.5)
        rotate(0, 0.1, str(movingDitto[0]))

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
