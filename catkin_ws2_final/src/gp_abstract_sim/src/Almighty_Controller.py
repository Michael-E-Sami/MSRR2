#!/usr/bin/env python

import math
# import sys
import rospy
from geometry_msgs.msg import Point32, Twist
from matplotlib import pyplot as plt
#from mybot_gazebo.msg import PlanarOdometry, PlanarPose, PlanarTwist
from gp_abstract_sim.msg import path_points
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# plt.ion()

###########################################################################

speed = Twist()

current_x = 0.0
current_y = 0.0
current_yaw = 0.0

currentPath = path_points()
size = 0

delta_x = 0.0
delta_y = 0.0
angle_to_goal = 0.0
euc = 0.0

new_goal_x = 0.0
new_goal_y = 0.0
new_goal_yaw = 0.0

perf_x = []
perf_y = []

pose_achieved = False

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


def pathAcquisition(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard ")
    global currentPath
    global size

    currentPath.x = data.x
    currentPath.y = data.y
    currentPath.yaw = data.yaw
    size = len(currentPath.x)

###########################################################################

def main():
    print("All Might Controller!")

    rospy.init_node('All_Might_Controller',
                    anonymous=True, disable_signals=True)

    #rospy.Subscriber("PlanarOdometry", PlanarOdometry, callback)
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.Subscriber("AStarPath", path_points, pathAcquisition)

    speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

    # rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.wait_for_message("AStarPath", path_points, timeout=10)
        #rospy.wait_for_message("PlanarOdometry", PlanarOdometry, timeout=10)
        rospy.wait_for_message("/odom", Odometry, timeout=10)
        while size == 0 and not rospy.is_shutdown():
            print("Waiting For Data...")

        for i in range(1, size):
            global speed

            new_goal_x = currentPath.x[i]
            new_goal_y = currentPath.y[i]

            perf_x.append(current_x)
            perf_y.append(current_y)

            delta_x = new_goal_x - current_x
            delta_y = new_goal_y - current_y

            euc = math.sqrt((pow(delta_x, 2))+(pow(delta_y, 2)))

            while euc > 0.05 and not rospy.is_shutdown():
                delta_x = new_goal_x - current_x
                delta_y = new_goal_y - current_y

                euc = math.sqrt((pow(delta_x, 2))+(pow(delta_y, 2)))

                angle_to_goal = math.atan2(delta_y, delta_x)

                #print("Path length: "),
                #print(size-1)
                #print("Path Point: "),
                #print(i)

                #print("Euc = sqrt( "),
                #print(pow(delta_x, 2)),
                #print(" + "),
                #print(pow(delta_y, 2)),
                #print(" ) = "),
                #print(euc)

                #print("Approaching Checkpoint")

                #print("(Ang.to.goal)"),
                #print(angle_to_goal),
                #print(" - (cnt.yaw)"),
                #print(current_yaw),
                #print(" = "),
                print(abs(angle_to_goal - current_yaw))

                if abs(angle_to_goal - current_yaw) > 0.1:
                    #print("Angular Velocity")
                    #print(" ")
                    speed.linear.x = 0.0
                    if ((angle_to_goal - current_yaw) >= 0):
                       #speed.angular.z = -0.1
                       speed.angular.z = 1
                    else:
                       #speed.angular.z = 0.1
                        speed.angular.z = -1


                else:
                    #print("Angular Velocity")
                    #print(" ")
                    #speed.linear.x = 0.4
                    speed.linear.x = 0.8
                    speed.angular.z = 0.0
                speed_pub.publish(speed)

        perf_x.append(current_x)
        perf_y.append(current_y)

        yaw_check = currentPath.yaw - current_yaw

        #print("Crnt. Yaw: "),
        #print(current_yaw)

        #print("(Goal Yaw)"),
        #print(currentPath.yaw),
        #print(" - (Crnt. Yaw)"),
        #print(current_yaw),
        #print(" = "),
        #print(abs(yaw_check))
        #0.05
        while (abs(yaw_check) > 0.05) and not rospy.is_shutdown():
            yaw_check = currentPath.yaw - current_yaw
            #print("(Achieving Required Yaw)")
            #print(" ")

            #print("(Goal Yaw)"),
            #print(currentPath.yaw),
            #print(" - (Crnt. Yaw)"),
            #print(current_yaw),
            #print(" = "),
            #print(abs(currentPath.yaw - current_yaw))
            #print(" ")

            speed.linear.x = 0.0
            if ((yaw_check) >= 0):
                #speed.angular.z = -0.1
                speed.angular.z = 0.5

            else:
                #speed.angular.z = 0.1
                speed.angular.z = -0.5
            speed_pub.publish(speed)

        pose_achieved = True

        if(pose_achieved):
            print(" ")
            print("Goal Reached! Take care now. Bye bye then")
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            speed_pub.publish(speed)

            print(perf_x)
            print(" ")
            print(perf_y)
            print(" ")

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
            #plt.close(1)

            while not rospy.is_shutdown():
                rospy.spin()


if __name__ == '__main__':
    main()
