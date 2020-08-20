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
from AStar_v6 import *

###########################################################################

whichDitto = [0, 0]
isattached =False
currentX = [0, 0]
currentY = [0, 0]
currentYawDeg = [0, 0]
currentYawRad = [0, 0]
str1=""
pub = {}

###########################################################################
def callback0(msg,args):
        global str1
        global isattached
        str1=msg.data
        cut = str1.split('#')
        if cut[1].find('magnet') <0:
            return
        if cut[0].find('magnet') <0:
            return
        first = cut[0].split('::')
        second = cut[1].split('::')
        first_body = first[1].split('_')
        first_pol=first[2] # north or south
        first_body= first_body[0]  # body or front
        if first_body == 'front':
            first_body='front_face'
        second_body=second[1].split('_')
        second_pol=second[2] # north or south
        second_body=second_body[0] # body or front
        if second_body == 'front':
            second_body='front_face'
        #print first[1].find('north')
        if first_pol == second_pol:
            tree= first[0] + "/"+first_body +"#" + second[0]+"/"+second_body
            if first[0]== "ditto"+ movingDitto[0] and second[0]== "ditto"+ stableDitto[0]:
                isattached =True
                print'line 54'
            #else:
               # print'line 55'
        else:
            tree= first[0] + "/"+first_body +"#" + second[0]+"/"+second_body
            if first[0]== "ditto"+ stableDitto[0] and second[0]== "ditto"+ movingDitto[0]:
                isattached =True
                print'line 61'
            #else:
              #  print 'line 61'
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

###############################################################################
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
    print 'Yaw goal = ',yaw_goal
    file1 = open(completeName, "r")
    list_of_lines = file1.readlines()
    for i in range(0, len(dittoNum)):
        list_of_lines[int(dittoNum[i])] = (str(yaw_goal))+ "\n"

    file1 = open(completeName, "w")
    file1.writelines(list_of_lines)
    file1.close()

    while start:
        print('yaw goal - current yaw = ', yaw_goal - currentYawDeg[indexRot])
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
def forwardFnDist(Distance, Omega, dittoNum):
    global start2, currentX, currentY, currentYawRad, currentYawDeg, whichDitto
    start2 = True
    FrwDiff_x = 0
    FrwDiff_y = 0
    eucFrw = 0
    indexFrw = 69  # Error Num
    distance = float(Distance)
    print("Forwarding")

    # Mafrood Neb3at dittoNum As a String

    for n in range(0, len(whichDitto)):
        print ' n = ', n
        print 'which ditto', whichDitto[n]
        if whichDitto[n] == int(dittoNum[0]):
            indexFrw = n

    # print 'indexFrw', indexFrw
    # print ' '
    # print 'dittoNum', dittoNum[0]
    # print 'which ditto 1 ', whichDitto[1]
    goalx= distance *cos(currentYawRad[indexFrw]) + currentX[indexFrw]
    goaly= distance *sin(currentYawRad[indexFrw])  + currentY[indexFrw]
    while start2:
        FrwDiff_x = goalx - currentX[indexFrw]
        FrwDiff_y = goaly - currentY[indexFrw]

        eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

        if eucFrw >= 0.01 and isattached==False:
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
        

##########################################################################

def main():
    global movingDitto, stableDitto, multiDitto
    perf_x = []
    perf_y = []
    perf_x_round = []
    perf_y_round = []
    angleToNext_old = 0
    plt.ion()

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

    rospy.Subscriber("/ditto1/sensor_data",String,callback0,"ditto1")
    rospy.Subscriber("/ditto2/sensor_data",String,callback0,"ditto2")
    rospy.Subscriber("/ditto0/sensor_data",String,callback0,"ditto0")
    ##############
    # HENA HALEF EL ROBOT EL TANY W AFARMELO, W momken ADELO ZA2A Odam
    lastX = currentPath.x[size - 1]
    lastY = currentPath.y[size - 1]

    bLastX = currentPath.x[size - 2]
    bLastY = currentPath.y[size - 2]

    lastAngle = atan2((lastY - bLastY), (lastX - bLastX))*180/math.pi
    lastAngleRad = atan2((lastY - bLastY), (lastX - bLastX))

    # Howa Hena Lazem Ab3at el Difference
    rotate(lastAngle, 0.05, str(stableDitto))
    rospy.sleep(0.5)
    rotate(0, 0.05, str(stableDitto))
    rospy.sleep(0.5)
    rotate(0, 0.05, str(stableDitto))

    beforeMovingX=currentX[1]
    beforeMovingY=currentY[1]

    print' beforeMovingX= ', beforeMovingX
    print' beforeMovingY= ', beforeMovingY
    tempX1 = (0.16*cos(lastAngleRad)) + currentX[1]
    tempY1 = (0.16*sin(lastAngleRad)) + currentY[1]

    forwardFn(tempX1, tempY1, 0.15, str(stableDitto))

    ##############
    # Going Behind The Docked Dittos (La Mo2a5za)
    # behindX = currentX[1] - (0.5*cos(currentYawRad[1]))
    # behindY = currentY[1] - (0.5*sin(currentYawRad[1]))
    # perf_x.append(currentX[0])
    # perf_y.append(currentY[0])
    # print'behindX = ' , behindX
    # print'behindY = ' , behindY

    # del_x = behindX - currentX[0]
    # del_y = behindY - currentX[0]

    # print ' del_x = ',del_x
    # print ' del_y = ',del_y

    # # while del_euc > 0.01 and not rospy.is_shutdown():
    # #     del_x = behindX - currentX[0]
    # #     del_y = behindY - currentX[0]

    # #     del_euc = sqrt((pow(del_x, 2))+(pow(del_y, 2)))

    # angle_to_goal = atan2(del_y, del_x)*180/math.pi
    # print 'Angle To Goal = ', angle_to_goal
    # rotate(angle_to_goal-currentYawDeg[0], 0.025, str(movingDitto[0]))
    # rospy.sleep(0.5)
    # rotate(0, 0.025, str(movingDitto[0]))
    # rospy.sleep(0.5)
    # rotate(0, 0.025, str(movingDitto[0]))

    # forwardFn(behindX, behindY, 0.15, str(movingDitto[0]))
    # perf_x.append(currentX[0])
    # perf_y.append(currentY[0])

    #     if abs(angle_to_goal - currentYawRad[0]) > 0.0174533:
    #         if ((angle_to_goal - currentYawRad[0]) >= 0):
    #             pub["ditto" + str(whichDitto[0]) + "/left_wheel_speed"].publish(-0.025)
    #             pub["ditto" + str(whichDitto[0]) + "/right_wheel_speed"].publish(0.025)

    #         else:
    #             pub["ditto" + str(whichDitto[0]) + "/left_wheel_speed"].publish(0.025)
    #             pub["ditto" + str(whichDitto[0]) + "/right_wheel_speed"].publish(-0.025)

    #     else:
    #         pub["ditto" + str(whichDitto[0]) + "/left_wheel_speed"].publish(0.15)
    #         pub["ditto" + str(whichDitto[0]) + "/right_wheel_speed"].publish(0.15)

    # pub["ditto" + str(whichDitto[0]) + "/left_wheel_speed"].publish(0)
    # pub["ditto" + str(whichDitto[0]) + "/right_wheel_speed"].publish(0)

    ##############

    while not rospy.is_shutdown():
        behindX1 = currentX[1] - (0.1*cos(currentYawRad[1]))
        behindY1 = currentY[1] - (0.1*sin(currentYawRad[1]))
        ox, oy = [], []

        for i in range(-2, 2):
            ox.append(i)
            oy.append(-2.0)
        for i in range(-2, 2):
            ox.append(2.0)
            oy.append(i)
        for i in range(-2, 2):
            ox.append(i)
            oy.append(2.0)
        for i in range(-2, 2):
            ox.append(-2.0)
            oy.append(i)
        
        plt.close('all')

        print("Planning, Phase 2.....")
       
       
        a_star = AStarPlanner(ox, oy, 0.1, 0.07)
       
        rx, ry = a_star.planning(currentX[0], currentY[0], beforeMovingX,beforeMovingY)
        rx.reverse()
        ry.reverse()
        rx = [round(num, 3) for num in rx]
        ry = [round(num, 3) for num in ry]
        print ' rx = ',rx
        print ' ry = ',ry
        sizeTwo = len(rx)
        rx[sizeTwo-1]= beforeMovingX
        ry[sizeTwo-1]= beforeMovingY

        print(" ")
        
        for i in range(1, sizeTwo):

            nextX = rx[i]
            nextY = ry[i]

            perf_x.append(currentX[0])
            perf_y.append(currentY[0])

            delta_x = nextX - currentX[0]
            delta_y = nextY - currentY[0]

            distToNext = sqrt((pow(delta_x, 2)) + (pow(delta_y, 2)))

            angleToNext = atan2(delta_y, delta_x)*180/math.pi
            angleToNext_diff = angleToNext - currentYawDeg[0]
            if angleToNext_diff >180:
                angleToNext_diff =angleToNext_diff-360
            elif angleToNext_diff <-180:
                angleToNext_diff=angleToNext_diff+360
            elif angleToNext_diff >=0.01:
                rotate(0, 0.05, str(movingDitto[0]))
            elif angleToNext_diff <=-0.01:
                rotate(0, 0.05, str(movingDitto[0]))
            #angleToNext_old = angleToNext
            print ' Angle to Next =', angleToNext
            print ' Angle to Next diff =', angleToNext_diff
            rospy.sleep(20)
            # print 'RX = ', rx[i]
            # print 'RY = ', ry[i]
            # print ' '
            # print 'Moving Ditto Actual Yaw (Deg) = ', currentYawDeg[0]
            # print 'Angle to Next Point = ', angleToNext
            # print 'Old Angle to Next = ', angleToNext_old
            # print 'Diff. Between Them = ', angleToNext_diff
            # print ' '
            # print 'For Loop : Index = ', i
            # print 'RX = ', rx[i]
            # print 'RY = ', ry[i]
            # print ' '
            # print 'Current X[0] = ', currentX[0]
            # print 'Current Y[0] = ', currentY[0]
            # if i== size-1:
            #     rospy.sleep(10)

            # ROTATE
           
            rotate(angleToNext_diff, 0.05, str(movingDitto[0]))
            rospy.sleep(0.5)
            rotate(0, 0.05, str(movingDitto[0]))
            rospy.sleep(0.5)
            rotate(0, 0.05, str(movingDitto[0]))

            print 'Current Yaw Rad [0] = ', currentYawRad[0]
            print 'Current Yaw Deg [0] = ', currentYawDeg[0]


            # FORWARD
            # tempX0 = (distToNext*cos(currentYawRad[0])) + currentX[0]
            # tempY0 = (distToNext*sin(currentYawRad[0])) + currentY[0]
            # print 'tempX0 =', tempX0
            # print 'tempY0 =', tempY0
            # print 'next x = ', nextX
            # print 'next y = ', nextY
            forwardFn(nextX, nextY, 0.3, str(movingDitto[0]))
            # forwardFn(distToNext, 0.25, int(movingDitto))
        if isattached == True:
            break
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

        j = currentYawDeg[1]-currentYawDeg[0]
        print 'Last Angle - Current Yaw Deg [0] = ', j

        rotate(j, 0.025, str(movingDitto[0]))
        rospy.sleep(0.5)
        rotate(0, 0.1, str(movingDitto[0]))
        rospy.sleep(0.5)
        rotate(0, 0.1, str(movingDitto[0]))
        while isattached ==False:
            print 'moving forward again'
            forwardFnDist(0.02,0.02,movingDitto[0])

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
