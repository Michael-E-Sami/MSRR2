#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point, Point32, Pose, PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import math
from math import *
# Mostly Useless Imports
from std_msgs.msg import Header, String, UInt16
import tf
from std_msgs.msg import Float32
# ---------------------------------------------------------------------------------

size = 0

yoda = Odometry()
tag = 0
sock = AprilTagDetectionArray()
odo_tr = TransformStamped()
flag = [False for i in range(10)]
i=0
# ---------------------------------------------------------------------------------
def move(omega,left,right):

        left.publish(omega)
        right.publish(omega) 

#def brake(left,right):

#        left.publish(0)
#        right.publish(0)

def callback(data):
    global size
    global yoda
    global tag
    global sock
    global i
    global odo_tr
    br = tf.TransformBroadcaster()
    sock = data
    size = len(sock.detections)
    print(size)
    if size > 0:
        for i in range(0,size):
            print("Index: "),
            print(i)
            tag = sock.detections[i].id[0]
            print("Tag ID: "),
            print(tag)
            if tag == 0:
                print("THIS IS ID0::")
                if flag[0] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000)
                    flag[0] == True
                move(0.5,left_pub,right_pub)
            elif tag == 1:
                print("THIS IS ID1::")
                if flag[1] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 
                    flag[1] == True
                move(0.5,left_pub,right_pub)

            elif tag == 2:
                print("THIS IS ID2::")
                if flag[2] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[2] == True
                move(0.5,left_pub,right_pub)   
            elif tag == 3:
                print("THIS IS ID3::")
                if flag[3] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[3] == True
                move(0.5,left_pub,right_pub)

            elif tag == 4:
                if flag[4] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[4] == True
                move(0.5,left_pub,right_pub)

            elif tag == 5:
                if flag[5] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[5] == True
                move(0.5,left_pub,right_pub)
   

            elif tag == 6:
                print("THIS IS 6:")
                if flag[6] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[6] == True
                move(0.5,left_pub,right_pub)

            elif tag == 7:
                if flag[7] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[7] == True

                move(0.5,left_pub,right_pub)
            elif tag == 8:
                if flag[8] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[8] == True

                move(0.5,left_pub,right_pub)
            elif tag == 9:
                if flag[9] == False:
                    left_pub = rospy.Publisher('/ditto'+i+'/left_wheel_speed', Float32, queue_size=1000)
                    right_pub = rospy.Publisher('/ditto'+i+'/right_wheel_speed', Float32, queue_size=1000) 

                    flag[9] == True
                move(0.5,left_pub,right_pub)
        

# ---------------------------------------------------------------------------------


def py_accu_check():
    rospy.init_node('movecam_hybrid', anonymous=True)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)

    rospy.spin()

# ---------------------------------------------------------------------------------


if __name__ == '__main__':
    py_accu_check()
