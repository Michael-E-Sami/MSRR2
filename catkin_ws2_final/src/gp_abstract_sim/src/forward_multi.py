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
from std_msgs.msg import Float32
##################################################################
x = 0.0
y = 0.0
theta = 0.0
#ar[10]=0
###############################################

def move(omega,left,right):

        left.publish(omega)
        right.publish(omega) 

def brake(left,right):

        left.publish(0)
        right.publish(0) 
#############################################

def callback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y,rot_q.z, rot_q.w])

###########################################################################    

def main():
    rospy.init_node('tag_to_odom', anonymous=True)
    ronum=str(input("please enter no of robots"))
    ronumi=int(ronum)
    #for i in range (ronum-1):
    #    numove=str(input("please enter"+i+"robot"))
    #    left_pub1 = rospy.Publisher('/ditto'+numove[i]+'/left_wheel_speed', Float32, queue_size=1000)
    #    right_pub1 = rospy.Publisher('/ditto'+numove[i]+'/right_wheel_speed', Float32, queue_size=1000)

    r = rospy.Rate(10)

    distance = float(input('Enter your desired distnce: '))
    goal = Point()
    goal.x=distance  *cos(-1*theta)  +x
    goal.y=distance  *sin(-1*theta) +y

    while not rospy.is_shutdown():
        diff_x = goal.x - x # goal - current x
        diff_y = goal.y - y
        #angle_to_goal = atan2(diff_y, diff_x)
        euc = sqrt(pow(diff_x,2)+pow(diff_y,2))
        print'goal.x= ',goal.x
        print'goal.y= ',goal.y
        print'euc= ', euc
        print'Theta= ',theta
        if euc>0.05:
            if distance>0:
                for i in range (ronumi-1):
                    numove=str(input("please enter"+str(i)+"robot"))
                    left_pub = rospy.Publisher('/ditto'+numove+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+numove+'/right_wheel_speed', Float32, queue_size=1000)
                    move(0.5,left_pub,right_pub)
                    break
            else:
                for i in range (ronumi-1):
                    numove=str(input("please enter"+str(i)+"robot"))
                    left_pub = rospy.Publisher('/ditto'+numove+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+numove+'/right_wheel_speed', Float32, queue_size=1000)
                    move(-0.5,left_pub,right_pub)
                    break
        else: 
            for i in range (ronumi-1):
                numove=str(input("please enter"+str(i)+"robot"))
                left_pub = rospy.Publisher('/ditto'+numove+'/left_wheel_speed', Float32, queue_size=1000)
                right_pub = rospy.Publisher('/ditto'+numove+'/right_wheel_speed', Float32, queue_size=1000)
                brake(left_pub,right_pub)
                break
    while not rospy.is_shutdown():
        rospy.spin()   

if __name__ == '__main__':
    main()