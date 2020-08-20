#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point, Point32, Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
# Mostly Useless Imports
from std_msgs.msg import Header, String, UInt16
import tf
# ---------------------------------------------------------------------------------

size = 0

yoda = Odometry()
tag = 0
sock = AprilTagDetectionArray()
odo_tr = TransformStamped()
flag = [False for i in range(10)]
i=0
# ---------------------------------------------------------------------------------


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
                    ditto0_pub = rospy.Publisher(
                        "ditto0/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[0] == True
           
                yoda.child_frame_id = "ditto0"
                yoda.header.frame_id="root"
                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w
                
                odo_tr.header.stamp="now"
                odo_tr.header.frame_id="root"
                odo_tr.child_frame_id="camera_link"
                odo_tr.transform.translation.x=yoda.pose.pose.position.x
                odo_tr.transform.translation.y=yoda.pose.pose.position.y
                odo_tr.transform.translation.z=1.2
                odo_tr.transform.rotation.x=0
                odo_tr.transform.rotation.y=3.141592654
                odo_tr.transform.rotation.z=0
                br.sendTransform((0.25,0.25,1.2),tf.transformations.quaternion_from_euler(0,3.141592654,0),
                    rospy.Time.now(),"camera_link","world")
                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
                resp = serv('ditto0', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z

                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto0_pub.publish(yoda)
                print("Index 0 AGAIN:: "),
                print(i)
                print("0 pos: "+str(yoda.pose.pose.position.x)+","+str(yoda.pose.pose.position.y)+","+str(yoda.pose.pose.position.z)+", 0 ori: "+str(yoda.pose.pose.orientation.x))
                print("---------------------------------------END0-----------------------------------------")
            elif tag == 1:
                print("THIS IS ID1::")
                if flag[1] == False:
                    ditto1_pub = rospy.Publisher(
                        "ditto1/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[1] == True

                yoda.child_frame_id = "ditto1"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
                resp = serv('ditto1', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z

                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto1_pub.publish(yoda)
                print("Index 1 AGAIN:: "),
                print(i)
                print("1 pos: "+str(yoda.pose.pose.position.x)+","+str(yoda.pose.pose.position.y)+","+str(yoda.pose.pose.position.z)+", 1 ori: "+str(yoda.pose.pose.orientation.x))
                print("---------------------------------------END1-----------------------------------------")
            elif tag == 2:
                print("THIS IS ID2::")
                if flag[2] == False:
                    ditto2_pub = rospy.Publisher(
                        "ditto2/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[2] == True
                yoda.header.frame_id="root"
                yoda.child_frame_id = "ditto2"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
                resp = serv('ditto2', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z

                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto2_pub.publish(yoda)
                print("Index 2 AGAIN:: "),
                print(i)
                print("2 pos: "+str(yoda.pose.pose.position.x)+", 2 ori: "+str(yoda.pose.pose.orientation.x))
                print("---------------------------------------END2-----------------------------------------")
            # elif tag == 1 and tag2==2:
            #     if flag[1] and flag[2] == False:
            #         ditto1_pub = rospy.Publisher(
            #             "ditto1/odom", Odometry, queue_size=1000)
            #         ditto2_pub = rospy.Publisher(
            #             "ditto2/odom", Odometry, queue_size=1000)
            #         rospy.wait_for_service('gazebo/get_model_state')
            #     flag[1] = True
            #     flag[2] = True
            #     print("THIS IS ID1 &2 SIMUL:")
            #     print("")
            #     yoda.child_frame_id = "ditto1"

            #     yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
            #     yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
            #     yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

            #     yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
            #     yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
            #     yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
            #     yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

            #     serv = rospy.ServiceProxy(
            #         'gazebo/get_model_state', GetModelState)
            #     resp = serv('ditto1', 'ground_plane')

            #     yoda.twist.twist.linear.x = resp.twist.linear.x
            #     yoda.twist.twist.linear.y = resp.twist.linear.y
            #     yoda.twist.twist.linear.z = resp.twist.linear.z

            #     yoda.twist.twist.angular.x = resp.twist.angular.x
            #     yoda.twist.twist.angular.y = resp.twist.angular.y
            #     yoda.twist.twist.angular.z = resp.twist.angular.z

            #     ditto1_pub.publish(yoda)

            #     yoda2.child_frame_id = "ditto2"

            #     yoda2.pose.pose.position.x = sock.detections[j].pose.pose.pose.position.x
            #     yoda2.pose.pose.position.y = sock.detections[j].pose.pose.pose.position.y
            #     yoda2.pose.pose.position.z = sock.detections[j].pose.pose.pose.position.z

            #     yoda2.pose.pose.orientation.x = sock.detections[j].pose.pose.pose.orientation.x
            #     yoda2.pose.pose.orientation.y = sock.detections[j].pose.pose.pose.orientation.y
            #     yoda2.pose.pose.orientation.z = sock.detections[j].pose.pose.pose.orientation.z
            #     yoda2.pose.pose.orientation.w = sock.detections[j].pose.pose.pose.orientation.w

            #     resp2 = serv('ditto2', 'ground_plane')

            #     yoda2.twist.twist.linear.x = resp2.twist.linear.x
            #     yoda2.twist.twist.linear.y = resp2.twist.linear.y
            #     yoda2.twist.twist.linear.z = resp2.twist.linear.z

            #     yoda2.twist.twist.angular.x = resp2.twist.angular.x
            #     yoda2.twist.twist.angular.y = resp2.twist.angular.y
            #     yoda2.twist.twist.angular.z = resp2.twist.angular.z

            #     ditto2_pub.publish(yoda2)                                
            elif tag == 3:
                print("THIS IS ID3::")
                if flag[3] == False:
                    ditto3_pub = rospy.Publisher(
                        "ditto3/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[3] == True

                yoda.child_frame_id = "ditto3"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
                resp = serv('ditto3', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z

                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto3_pub.publish(yoda)
                print("Index AGAIN3:: ")
                print(i)
            elif tag == 4:
                if flag[4] == False:
                    ditto4_pub = rospy.Publisher(
                        "ditto4/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[4] == True

                yoda.child_frame_id = "ditto4"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
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
                    ditto5_pub = rospy.Publisher(
                        "ditto5/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[5] == True

                yoda.child_frame_id = "ditto5"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
                resp = serv('ditto5', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z

                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto5_pub.publish(yoda)

            elif tag == 6:
                print("THIS IS 6:")
                if flag[6] == False:
                    ditto6_pub = rospy.Publisher(
                        "ditto6/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[6] == True

                yoda.child_frame_id = "ditto6"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
                resp = serv('ditto6', 'ground_plane')

                yoda.twist.twist.linear.x = resp.twist.linear.x
                yoda.twist.twist.linear.y = resp.twist.linear.y
                yoda.twist.twist.linear.z = resp.twist.linear.z

                yoda.twist.twist.angular.x = resp.twist.angular.x
                yoda.twist.twist.angular.y = resp.twist.angular.y
                yoda.twist.twist.angular.z = resp.twist.angular.z

                ditto6_pub.publish(yoda)
                print("Index 6 AGAIN:: "),
                print(i)
                print("6 pos: "+str(yoda.pose.pose.position.x)+", 6 ori: "+str(yoda.pose.pose.orientation.x))
                print("---------------------------------------END6-----------------------------------------")
            elif tag == 7:
                if flag[7] == False:
                    ditto7_pub = rospy.Publisher(
                        "ditto7/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[7] == True

                yoda.child_frame_id = "ditto7"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
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
                    ditto8_pub = rospy.Publisher(
                        "ditto8/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[8] == True

                yoda.child_frame_id = "ditto8"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
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
                    ditto9_pub = rospy.Publisher(
                        "ditto9/odom", Odometry, queue_size=1000)
                    rospy.wait_for_service('gazebo/get_model_state')
                    flag[9] == True

                yoda.child_frame_id = "ditto9"

                yoda.pose.pose.position.x = sock.detections[i].pose.pose.pose.position.x
                yoda.pose.pose.position.y = sock.detections[i].pose.pose.pose.position.y
                yoda.pose.pose.position.z = sock.detections[i].pose.pose.pose.position.z

                yoda.pose.pose.orientation.x = sock.detections[i].pose.pose.pose.orientation.x
                yoda.pose.pose.orientation.y = sock.detections[i].pose.pose.pose.orientation.y
                yoda.pose.pose.orientation.z = sock.detections[i].pose.pose.pose.orientation.z
                yoda.pose.pose.orientation.w = sock.detections[i].pose.pose.pose.orientation.w

                serv = rospy.ServiceProxy(
                    'gazebo/get_model_state', GetModelState)
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
    rospy.init_node('tag_to_odom', anonymous=True)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)

    rospy.spin()

# ---------------------------------------------------------------------------------


if __name__ == '__main__':
    py_accu_check()
