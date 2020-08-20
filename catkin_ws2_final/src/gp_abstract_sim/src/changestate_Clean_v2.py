#!/usr/bin/env python

import math
from math import atan2, pow, sqrt

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point32, Pose, Twist
from gp_abstract_sim.msg import path_points
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String


from matplotlib import pyplot as plt

###########################################################################
sensor =None
isdetected =False
speed = Twist()
state_msg = ModelState()
gestate = GetModelState()
new_data = Odometry()
currentPath = path_points()
counter =0

current_x = 0.0
current_y = 0.0
current_yaw = 0.0
roll = 0.0
pitch = 0.0

flagdamn = False
flag = False

gx = 0.0
gy = 0.0
gyaw = 0.0

flagmove = False

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

amp_resolution = False
high_res_euc = 0.05
low_res_euc = 0.1
euc_condition = 0.0

dittoS = 0
dittoG = 0

###########################################################################


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

def callback0(msg):
    global isdetected 
    #isdetected =True
    global str1
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
        isdetected =True
    else:
       
        tree= first[0] + "/"+first_body +"#" + second[0]+"/"+second_body
        isdetected =True
            # ditto1/body#ditto2/front_face
    print tree

def statecall(dittoS, dittoG):
    if flag == False:
        rospy.wait_for_service('gazebo/get_model_state')
        flag == True

    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    get_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    gesp2 = get_state('ditto'+dittoG, 'ground_plane')
    gesp1 = get_state('ditto'+dittoS, 'ground_plane')
    print("gesp2::"+str(gesp2.pose.position.x)+","+str(gesp2.pose.position.y)+"," +
          str(gesp2.pose.orientation.x)+","+str(gesp2.pose.orientation.y)+","+str(gesp2.pose.orientation.w))
    print(" ")bv
    print("gesp1 :"+str(gesp1.pose.position.x)+","+str(gesp1.pose.position.y)+"," +
          str(gesp1.pose.orientation.x)+","+str(gesp1.pose.orientation.y)+","+str(gesp1.pose.orientation.w))
    state_msg.model_name = ("ditto"+dittoS)

    state_msg.reference_frame = 'ground_plane'

    state_msg.pose.position.x = gesp1.pose.position.x
    state_msg.pose.position.y = gesp1.pose.position.y
    state_msg.pose.position.z = gesp1.pose.position.z
    state_msg.pose.orientation.x = gesp2.pose.orientation.x
    state_msg.pose.orientation.y = gesp2.pose.orientation.y
    state_msg.pose.orientation.z = gesp2.pose.orientation.z
    state_msg.pose.orientation.w = gesp2.pose.orientation.w
    state_msg.twist.linear.x = gesp2.twist.linear.x
    state_msg.twist.linear.y = gesp2.twist.linear.y
    state_msg.twist.linear.z = gesp2.twist.linear.z

    state_msg.twist.angular.x = gesp2.twist.angular.x
    state_msg.twist.angular.y = gesp2.twist.angular.y
    state_msg.twist.angular.z = gesp2.twist.angular.z
    sesp = set_state(state_msg)

    gesp1up = get_state("ditto"+dittoS, 'ground_plane')
    gesp2up = get_state("ditto"+dittoG, 'ground_plane')
    print("NEW gesp1 :"+str(gesp1up.pose.position.x)+","+str(gesp1up.pose.position.y)+"," +
          str(gesp1up.pose.orientation.x)+","+str(gesp1up.pose.orientation.y)+","+str(gesp1up.pose.orientation.w))
    print("NEW gesp2 :"+str(gesp2up.pose.position.x)+","+str(gesp2up.pose.position.y)+"," +
          str(gesp2up.pose.orientation.x)+","+str(gesp2up.pose.orientation.y)+","+str(gesp2up.pose.orientation.w))
    if(round(gesp1up.pose.orientation.x, 3) == round(gesp2.pose.orientation.x, 3) and round(gesp1up.pose.orientation.y, 3) == round(gesp2.pose.orientation.y, 3) and round(gesp1up.pose.orientation.w, 3) == round(gesp2.pose.orientation.w, 3)):
        flagdamn = True
        print("7ELWWWWWWWWWWWWW")
        print("7ELWWWWWWWWWW")
        print("7ELWWWWWWWW")
        print("7ELWWWWWW")

def pathAcquisition(data):
    global currentPath
    global size

    currentPath.x = data.x
    currentPath.y = data.y
    currentPath.yaw = data.yaw
    currentPath.dOne = data.dOne
    currentPath.dTwo = data.dTwo
    size = len(currentPath.x)

###########################################################################


def main():
    print("All Might Controller!")
    global counter
    rospy.init_node('All_Might_Controller',
                    anonymous=True, disable_signals=True)

    rospy.Subscriber("AStarPath", path_points, pathAcquisition)
    sub1= rospy.Subscriber("/ditto1/sensor_data",String,callback0)


    rospy.wait_for_message("AStarPath", path_points, timeout=10)
    while size == 0 and not rospy.is_shutdown():
        print("Waiting For Data...")

    global dittoS, dittoG
    dittoS = str(currentPath.dOne)
    dittoG = str(currentPath.dTwo)

    rospy.Subscriber("ditto" + dittoS + "/odom", Odometry, callbackS)
    rospy.Subscriber("ditto" + dittoG + "/odom", Odometry, callbackG)

    rospy.wait_for_message("ditto" + dittoS + "/odom", Odometry, timeout=10)
    rospy.wait_for_message("ditto" + dittoG + "/odom", Odometry, timeout=10)

    left_pubS = rospy.Publisher(
        "ditto" + dittoS + "/left_wheel_speed", Float32, queue_size=1000)
    right_pubS = rospy.Publisher(
        "ditto" + dittoS + "/right_wheel_speed", Float32, queue_size=1000)
    left_pubG = rospy.Publisher(
        "ditto" + dittoG + "/left_wheel_speed", Float32, queue_size=1000)
    right_pubG = rospy.Publisher(
        "ditto" + dittoG + "/right_wheel_speed", Float32, queue_size=1000)

    global amp_resolution, high_res_euc, low_res_euc, euc_condition

    while not rospy.is_shutdown():
        for i in range(1, size):
            if isdetected ==True:
                break
            new_goal_x = currentPath.x[i]
            new_goal_y = currentPath.y[i]

            perf_x.append(current_x)
            perf_y.append(current_y)

            delta_x = new_goal_x - current_x
            delta_y = new_goal_y - current_y

            euc = math.sqrt((pow(delta_x, 2))+(pow(delta_y, 2)))

            if size < 2:
                amp_resolution = True
                euc_condition = high_res_euc

            elif size > 1 and i >= size - 1:
                amp_resolution = True
                euc_condition = high_res_euc

            else:
                euc_condition = low_res_euc

            # if amp_resolution:
            #     brake(left_pubS, right_pubS)
            #     while (abs(current_yaw - gyaw) > 0.04) and (not rospy.is_shutdown()):
            #         # Might not work in case that they are on the opposite sides of the (-Pi,+Pi)
            #         print("Rotating Other Robot")
            #         print(" ")
            #         if ((current_yaw - gyaw) >= 0):
            #             rotateright(0.125, left_pubG, right_pubG)
            #         else:
            #             rotateleft(0.125, left_pubG, right_pubG)

            #         #print("STATECALL is being CALLED")
            #     brake(left_pubG, right_pubG)
            #     statecall(dittoS, dittoG)

                brake(left_pubG, right_pubG)
            while euc > euc_condition and not rospy.is_shutdown():
                delta_x = new_goal_x - current_x
                delta_y = new_goal_y - current_y
                euc = math.sqrt((pow(delta_x, 2))+(pow(delta_y, 2)))
                angle_to_goal = math.atan2(delta_y, delta_x)
                
                print("Amp Reso.? "),
                print(str(amp_resolution))
                print("Path length: "),
                print(size-1)
                print("Path Point: "),
                print(i)

                # print("Euc = sqrt( "),
                # print(pow(delta_x, 2)),
                # print(" + "),
                # print(pow(delta_y, 2)),
                # print(" ) = "),
                # print(euc)

                # print("Approaching Checkpoint")

                # print("(Ang.to.goal)"),
                # print(angle_to_goal),
                # print(" - (cnt.yaw)"),
                # print(current_yaw),
                # print(" = "),
                # print(abs(angle_to_goal - current_yaw))

                if abs(angle_to_goal - current_yaw) > 0.174533:
                    # print("Angular Velocity")
                    # print(" ")
                    if ((angle_to_goal - current_yaw) >= 0):
                        rotateright(0.2, left_pubS, right_pubS)
                    else:
                        rotateleft(0.2, left_pubS, right_pubS)

                else:
                    # print("Angular Velocity")
                    # print(" ")
                    move(0.5, left_pubS, right_pubS)

                if amp_resolution and (counter%5000==0):

                    brake(left_pubS, right_pubS)
                    while (abs(current_yaw - gyaw) > 0.087) and (not rospy.is_shutdown()):
                        # Might not work in case that they are on the opposite sides of the (-Pi,+Pi)
                        print("Rotating Other Robot")
                        print(" ")
                        if ((current_yaw - gyaw) >= 0):
                            rotateright(0.125, left_pubG, right_pubG)
                        else:
                            rotateleft(0.125, left_pubG, right_pubG)

                        #print("STATECALL is being CALLED")
                    brake(left_pubG, right_pubG)
                    statecall(dittoS, dittoG)


                counter= counter+1

                if isdetected ==True:
                    break
            counter =0



            if(flagdamn == True):
                print("FlagDamn is TRUE")

        perf_x.append(current_x)
        perf_y.append(current_y)

        pose_achieved = True

        if(pose_achieved):
            print(" ")
            print("Goal Reached! Take care now. Bye bye then")

            brake(left_pubS, right_pubS)
            brake(left_pubG, right_pubG)

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

            brake(left_pubS, right_pubS)
            brake(left_pubG, right_pubG)

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
