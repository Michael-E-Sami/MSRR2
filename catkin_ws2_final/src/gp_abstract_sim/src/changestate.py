#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Point32, Twist, Pose
from matplotlib import pyplot as plt
from gp_abstract_sim.msg import path_points
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelState
from math import sqrt, pow, atan2
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetModelState, SetModelState
###########################################################################

speed = Twist()
state_msg = ModelState()
gestate=GetModelState()
sestate=SetModelState()
new_data=Odometry()
current_x = 0.0
current_y = 0.0
current_yaw = 0.0
flagdamn=False
flag=False
gx = 0.0
gy = 0.0
gyaw = 0.0

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

def statecall():
    global flag, flagdamn
    #rot_q = data.pose.pose.orientation
    #(roll, pitch, gyaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #ditto1_pub = rospy.Publisher("ditto1/odom", Odometry, queue_size=1000)
    #ditto2_pub = rospy.Publisher("ditto2/odom", Odometry, queue_size=1000)
    if flag==False:
        set_state=rospy.Publisher("ditto1/odom", Odometry, queue_size=1000)
        rospy.wait_for_service('gazebo/get_model_state')
        flag=True
    if flag==True:
        #set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        get_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        #sesp = set_state( state_msg )
        gesp2 = get_state('ditto2','ground_plane')
        gesp1 = get_state('ditto1','ground_plane')
        print("gesp2::"+str(gesp2.pose.position.x)+","+str(gesp2.pose.position.y)+","+str(gesp2.pose.orientation.x)+","+str(gesp2.pose.orientation.y)+","+str(gesp2.pose.orientation.w))
        print(" ")
        print("gesp1 :"+str(gesp1.pose.position.x)+","+str(gesp1.pose.position.y)+","+str(gesp1.pose.orientation.x)+","+str(gesp1.pose.orientation.y)+","+str(gesp1.pose.orientation.w))
        #state_msg.model_name='ditto1'
        new_data.child_frame_id='ditto1'
        new_data.header.frame_id='ground_plane'
        #state_msg.reference_frame='ground_plane'
        new_data.pose.pose.position.x=gesp1.pose.position.x
        new_data.pose.pose.position.y=gesp1.pose.position.y
        new_data.pose.pose.position.z=gesp1.pose.position.z
        new_data.pose.pose.orientation.x=gesp2.pose.orientation.x
        new_data.pose.pose.orientation.y=gesp2.pose.orientation.y
        new_data.pose.pose.orientation.z=gesp2.pose.orientation.z
        new_data.pose.pose.orientation.w=gesp2.pose.orientation.w
    #state_msg.pose.position.x=gesp1.pose.position.x
    #state_msg.pose.position.y=gesp1.pose.position.y
    #state_msg.pose.position.z=gesp1.pose.position.z
    #state_msg.pose.orientation.x = gesp2.pose.orientation.x
    #state_msg.pose.orientation.y = gesp2.pose.orientation.y
    #state_msg.pose.orientation.z = gesp2.pose.orientation.z
    #state_msg.pose.orientation.w = gesp2.pose.orientation.w
    #state_msg.twist.linear.x = gesp2.twist.linear.x
    #state_msg.twist.linear.y = gesp2.twist.linear.y
    #state_msg.twist.linear.z = gesp2.twist.linear.z
                
    #state_msg.twist.angular.x = gesp2.twist.angular.x
    #state_msg.twist.angular.y = gesp2.twist.angular.y
    #state_msg.twist.angular.z = gesp2.twist.angular.z
        set_state.publish(new_data)
        gesp1up=get_state('ditto1','ground_plane')
        gesp2up=get_state('ditto2','ground_plane')
        print("NEW gesp1 :"+str(gesp1up.pose.position.x)+","+str(gesp1up.pose.position.y)+","+str(gesp1up.pose.orientation.x)+","+str(gesp1up.pose.orientation.y)+","+str(gesp1up.pose.orientation.w))  
        print("NEW gesp2 :"+str(gesp2up.pose.position.x)+","+str(gesp2up.pose.position.y)+","+str(gesp2up.pose.orientation.x)+","+str(gesp2up.pose.orientation.y)+","+str(gesp2up.pose.orientation.w))
        if(gesp1up.pose.orientation.x==gesp2.pose.orientation.x and gesp1up.pose.orientation.y==gesp2.pose.orientation.y  and gesp1up.pose.orientation.w==gesp2.pose.orientation.w ):
            flagdamn=True
    #    move(0.5,left_pubG,right_pubG)
    #new_data.child_frame_id = "ditto1"
    #new_data.pose.pose.position.orientation = rot_q
    #ditto1_pub.publish(data)
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

    rospy.init_node('All_Might_Controller',
                    anonymous=True, disable_signals=True)

    rospy.Subscriber("AStarPath", path_points, pathAcquisition)

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
    #dstate=rospy.Subscriber("/gazebo/model_states",Float32, queue_size=1000)
    #dx=rospy.Publisher("ditto2/left_wheel_speed",Float32, queue_size=1000)
    global amp_resolution, high_res_euc, low_res_euc, euc_condition

    while not rospy.is_shutdown():
        for i in range(1, size):
            new_goal_x = currentPath.x[i]
            new_goal_y = currentPath.y[i]

            perf_x.append(current_x)
            perf_y.append(current_y)

            delta_x = new_goal_x - current_x
            delta_y = new_goal_y - current_y

            euc = math.sqrt((pow(delta_x, 2))+(pow(delta_y, 2)))

            if size < 3:
                amp_resolution = True
                euc_condition = high_res_euc

            elif size > 2 and i >= size - 2:
                amp_resolution = True
                euc_condition = high_res_euc
                
            else:
                euc_condition = low_res_euc

            if amp_resolution:
                brake(left_pubS,right_pubS)
                while abs(current_yaw - gyaw) > 0.0872665 and not rospy.is_shutdown(): 
                    
                    # Might not work in case that they are on the opposite sides of the (-Pi,+Pi)
                    print("Rotating Other Robot")
                    print(" ")
                    if ((current_yaw - gyaw) >= 0):
                        rotateright(0.25, left_pubG, right_pubG)
                    else:
                        rotateleft(0.25, left_pubG, right_pubG)

                brake(left_pubG,right_pubG)
                
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

                print("Euc = sqrt( "),
                print(pow(delta_x, 2)),
                print(" + "),
                print(pow(delta_y, 2)),
                print(" ) = "),
                print(euc)

                print("Approaching Checkpoint")

                print("(Ang.to.goal)"),
                print(angle_to_goal),
                print(" - (cnt.yaw)"),
                print(current_yaw),
                print(" = "),
                print(abs(angle_to_goal - current_yaw))

                if abs(angle_to_goal - current_yaw) > 0.174533:
                    print("Angular Velocity")
                    print(" ") 
                    if ((angle_to_goal - current_yaw) >= 0):
                        rotateright(0.5, left_pubS, right_pubS)
                    else:
                        rotateleft(0.5, left_pubS, right_pubS)
                
                else:
                    print("Angular Velocity")
                    print(" ")
                    move(0.5, left_pubS, right_pubS)

                #if abs(angle_to_goal - current_yaw) == 0.1:
                #    brake(left_pubG,right_pubG)
                #    statecall()
                #if(flagdamn==True):
            #damn=statecall()
            #if(damn.sesp.pose.orientation.x==damn.gesp.pose.orientation.x and damn.sesp.pose.orientation.y==damn.gesp.pose.orientation.y and damn.sesp.pose.orientation.z==damn.gesp.pose.orientation.z and damn.sesp.pose.orientation.w==damn.gesp.pose.orientation.w ):
                    #print("AYWAAAAAAAAAAAA")
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
            brake(left_pubS,right_pubS)
            brake(left_pubG,right_pubG)
            #statec=rospy.Subscriber("ditto" + dittoG + "/odom", Odometry, statecall)
            #move(0.4,left_pubG,right_pubG)
            rospy.sleep(1)
            statecall()
            if(flagdamn==True):
            #damn=statecall()
            #if(damn.sesp.pose.orientation.x==damn.gesp.pose.orientation.x and damn.sesp.pose.orientation.y==damn.gesp.pose.orientation.y and damn.sesp.pose.orientation.z==damn.gesp.pose.orientation.z and damn.sesp.pose.orientation.w==damn.gesp.pose.orientation.w ):
                print("AYWAAAAAAAAAAAA")
                brake(left_pubG,right_pubG)
                brake(left_pubS,right_pubS)
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
