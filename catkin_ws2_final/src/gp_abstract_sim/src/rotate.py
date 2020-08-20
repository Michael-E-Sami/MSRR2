#!/usr/bin/env python
import math
import os.path
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32


#from mybot_gazebo.msg import PlanarOdometry, PlanarPose, PlanarTwist

###########################################################################

speed = Twist()

current_x = 0.0
current_y = 0.0
current_yaw = 0.0

yaw_goal = 0.0

pi = math.pi
##########################################################################
def move(omega, left, right):
    left.publish(omega)
    right.publish(omega)


def brake(left, right):
    left.publish(0)
    right.publish(0)


def rotateleft(omega, left, right):
    left.publish(omega)
    right.publish(-omega)


def rotateright(omega, left, right):
    left.publish(-omega)
    right.publish(omega)

###########################################################################

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard ")
    global current_x
    global current_y
    global current_yaw

    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, current_yaw) = euler_from_quaternion([rot_q.x, rot_q.y,rot_q.z, rot_q.w])
    
    current_yaw = current_yaw*(180/pi)

    if current_yaw < 0:
        current_yaw +=  360

###########################################################################

def main():
    print("All Might Controller!")

    rospy.init_node('rotate',
                    anonymous=True, disable_signals=True)

    rospy.Subscriber("/ditto2/odom", Odometry, callback)

    left_pubS = rospy.Publisher(
        "ditto2" + "/left_wheel_speed", Float32, queue_size=1000)
    right_pubS = rospy.Publisher(
        "ditto2" +  "/right_wheel_speed", Float32, queue_size=1000)

    # rate = rospy.Rate(20)

    if (current_x == 0 and current_y == 0 and current_yaw == 0):
            rospy.wait_for_message("/ditto2/odom", Odometry, timeout=10)

    save_path = os.path.dirname(__file__)
    name_of_file = "last_yaw_goal"
    completeName = os.path.join(save_path, name_of_file+".txt")         
    file1 = open(completeName, "r")
    first_yaw = file1.read()
    # print(first_yaw)
    first_yaw = float(first_yaw)
    file1.close()

    print("First Yaw: "),
    print(first_yaw)
    print(" ")

    g_yaw = input("Please Enter Rotation Value(Degrees) : ")
    print(" ")
    yaw_goal = g_yaw + first_yaw
    if yaw_goal < 0:
        yaw_goal = yaw_goal + 359.99
    elif yaw_goal > 359.99:
        yaw_goal = yaw_goal - 359.99

    file1 = open(completeName, "w")
    toFile = str(yaw_goal)
    file1.write(toFile)
    file1.close()


    while not rospy.is_shutdown():
        print("Current Yaw: "),
        print(current_yaw)
        print("Yaw Goal: "),
        print(yaw_goal)
        print("Abs(yaw_goal - current_yaw) = "),
        print(abs(yaw_goal - current_yaw))

        if abs(yaw_goal - current_yaw) > 0.1:
            print("Angular Velocity")
            print(" ")
            speed.linear.x = 0.0
            if (g_yaw >= 0):
                rotateright(0.2, left_pubS, right_pubS)
            else:
                rotateleft(0.2, left_pubS, right_pubS)
            
           
        
        else:
            brake(left_pubS,right_pubS)
            print(" ")
            print("Goal is Fucking Fucking Ya3ni...")
            print("Go To Hell!")
            rospy.signal_shutdown("Aho 8alasa keda.")

        

if __name__ == '__main__':
    main()
