#!/usr/bin/env python
import math
from math import cos, pow, sin, sqrt
import os.path
import rospy
from geometry_msgs.msg import Twist , Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
###########################################################################

speed = Twist()
current_x = 0.0
current_y = 0.0
current_yaw = 0.0
yaw_goal = 0.0
start = False
pi = math.pi

###########################################################################
frw_x = 0.0
frw_y = 0.0
frw_yaw = 0.0
start2=False
###########################################################################

def rotlecall(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard ")
    global current_x
    global current_y
    global current_yaw

    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, current_yaw) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    current_yaw = current_yaw*(180/pi)

    if current_yaw < 0:
        current_yaw += 360
##########################################################################
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
def rotate(omega, angle, ditto):
    print ('Rotating...')
    global current_x, current_y, current_yaw, ditto_yaw, start
    start =True
    current_x = 0
    current_y = 0
    current_yaw = 0
    ditto_yaw = 0

    ditstr = str(ditto)

    rospy.Subscriber("/ditto"+ditstr+"/odom", Odometry, rotlecall)

    left_pub = rospy.Publisher(
        "/ditto"+ditstr+"/left_wheel_speed", Float32, queue_size=1000)
    right_pub = rospy.Publisher(
        "/ditto"+ditstr+"/right_wheel_speed", Float32, queue_size=1000)

    rospy.wait_for_message("/ditto"+ditstr+"/odom", Odometry, timeout=10)

    save_path = os.path.dirname(__file__)
    name_of_file = "last_yaw_goal"
    completeName = os.path.join(save_path, name_of_file+".txt")
    file1 = open(completeName, "r")

    # Note that position == n-1 for the nth line.
    for position, line in enumerate(file1):
        # If line(position) desired is first line, then position == 0
        if position == ditto:
            ditto_yaw = line

    ditto_yaw = float(ditto_yaw)
    file1.close()

    yaw_goal = angle + ditto_yaw
    if yaw_goal < 0:
        yaw_goal = yaw_goal + 359.99
    elif yaw_goal > 359.99:
        yaw_goal = yaw_goal - 359.99

    file1 = open(completeName, "r")
    list_of_lines = file1.readlines()
    list_of_lines[int(ditto)] = (str(yaw_goal) + "\n")

    file1 = open(completeName, "w")
    file1.writelines(list_of_lines)
    file1.close()

    while start:
        print('yaw goal - current yaw := ', yaw_goal - current_yaw) 
        if abs(yaw_goal - current_yaw) > 0.1:
            
            if (angle > 0):
                print '2 := ', yaw_goal
                left_pub.publish(-omega)
                right_pub.publish(omega)
            elif angle < 0:
                print '3 := ', yaw_goal
                left_pub.publish(omega)
                right_pub.publish(-omega)
            elif angle == 0:
                omega = 0.025
                if yaw_goal - current_yaw > 0 :
                    print '4 := ', yaw_goal
                    left_pub.publish(-omega)
                    right_pub.publish(omega)
                else:
                    print '5 := ', yaw_goal
                    left_pub.publish(omega)
                    right_pub.publish(-omega)
   

        else:
            print '6 := ', yaw_goal
            left_pub.publish(0)
            right_pub.publish(0)
            start = False

######################################################################

def forwardFn(Distance, Omega, dittoNum):
    # rospy.init_node('planarMovement', anonymous=True, disable_signals=True)
    print(" ")
    print("Forwarding")
 
    global frw_x
    global frw_y
    global frw_yaw
    global start2
    start2 =True
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

    distance = float(Distance)
    goal = Point()
    goal.x= distance *cos(frw_yaw) + frw_x
    goal.y= distance *sin(frw_yaw)  + frw_y

    while start2:
        FrwDiff_x = goal.x - frw_x
        FrwDiff_y = goal.y - frw_y

        eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))

        # print'Dist. in X to goal.x= ', FrwDiff_x
        # print'Dist. in Y to goal.y= ', FrwDiff_y
        # print'euc= ', eucFrw

        while eucFrw > 0.05 :
            FrwDiff_x = goal.x - frw_x
            FrwDiff_y = goal.y - frw_y

            eucFrw = sqrt(pow(FrwDiff_x, 2)+pow(FrwDiff_y, 2))
            print'Dist. in X to goal.x= ', FrwDiff_x
            print'Dist. in Y to goal.y= ', FrwDiff_y
            print("Remaining Distance: "),
            print(eucFrw)
            print " "
            leftFrw.publish(omega)
            rightFrw.publish(omega)
        print('stop forward')
        print(" ")
        leftFrw.publish(0)
        rightFrw.publish(0)
        start2 =False
