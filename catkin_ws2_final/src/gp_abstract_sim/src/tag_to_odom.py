#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Point, Point32, Pose, PoseStamped, Quaternion
# Mostly Useless Imports
from std_msgs.msg import Header, String, UInt16
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState

# ---------------------------------------------------------------------------------
kolio=AprilTagDetectionArray()
size = 0

yoda = Odometry()

flag =  [False for i in range(10)] 

# ---------------------------------------------------------------------------------


def callback(data):
    print("ZEFT AWY")
    global size
    global yoda
    kolio=data
    size = len(kolio.detections)
    print ("size aho0000"+str(size))
    if size > 0:
        for i in range(0, size-1):
            tag = kolio.detections[i].id[i]
            print(tag)
            # SWITCH CASE
            if tag == 0:
                if flag[0] == False:
                    ditto0_pub = rospy.Publisher("ditto0/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[0] == True

                yoda.child_frame_id = "ditto0"

                yoda.pose.pose.position.x = kolio.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = kolio.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = kolio.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = kolio.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = kolio.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = kolio.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = kolio.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto0', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto0_pub.publish(yoda)

            elif tag == 1:
            
                if flag[1] == False:
                    ditto1_pub = rospy.Publisher("ditto1/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[1] == True

                yoda.child_frame_id = "ditto1"

                yoda.pose.pose.position.x = kolio.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = kolio.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = kolio.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = kolio.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = kolio.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = kolio.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = kolio.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto1', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto1_pub.publish(yoda)

            elif tag == 2:
                if flag[2] == False:
                    ditto2_pub = rospy.Publisher("ditto2/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[2] == True

                yoda.child_frame_id = "ditto2"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto2', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto2_pub.publish(yoda)

            elif tag == 3:
                if flag[3] == False:
                    ditto3_pub = rospy.Publisher("ditto3/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[3] == True

                yoda.child_frame_id = "ditto3"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto3', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto3_pub.publish(yoda)

            elif tag == 4:
                if flag[4] == False:
                    ditto4_pub = rospy.Publisher("ditto4/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[4] == True

                yoda.child_frame_id = "ditto4"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto4', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto4_pub.publish(yoda)

            elif tag == 5:
                if flag[5] == False:
                    ditto5_pub = rospy.Publisher("ditto5/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[5] == True

                yoda.child_frame_id = "ditto5"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto5', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto5_pub.publish(yoda)

            elif tag == 6:
                if flag[6] == False:
                    ditto6_pub = rospy.Publisher("ditto6/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[6] == True

                yoda.child_frame_id = "ditto6"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto6', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto6_pub.publish(yoda)

            elif tag == 7:
                if flag[7] == False:
                    ditto7_pub = rospy.Publisher("ditto7/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[7] == True

                yoda.child_frame_id = "ditto7"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto7', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto7_pub.publish(yoda)

            elif tag == 8:
                if flag[8] == False:
                    ditto8_pub = rospy.Publisher("ditto8/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[8] == True

                yoda.child_frame_id = "ditto8"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto8', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto8_pub.publish(yoda)

            elif tag == 9:
                if flag[9] == False:
                    ditto9_pub = rospy.Publisher("ditto9/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[9] == True

                yoda.child_frame_id = "ditto9"

                yoda.pose.pose.position.x = data.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = data.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = data.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = data.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = data.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = data.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = data.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
                resp = serv('ditto9', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z
                
                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto9_pub.publish(yoda)


# ---------------------------------------------------------------------------------


def py_accu_check():

    rospy.init_node('py_accu_check', anonymous=True)

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, callback)
    rospy.loginfo("7AAAAAAAAAARAAAAAAAAAAAAM")
    rospy.spin()


# ---------------------------------------------------------------------------------


if __name__ == '__main__':
    py_accu_check()
