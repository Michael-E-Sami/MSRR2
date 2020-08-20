#!/usr/bin/env python

# Mostly Useless Imports
from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Point32,
    Quaternion,
)
#------------------------------------


import rospy
from std_msgs.msg import String

from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection

def callback(data):
    rospy.loginfo("hello")
    
    # global objFramePrefix_
    # global distanceMax_
    # if len(data.objects.data) > 0:
    #     output = AprilTagDetectionArray()
    #     output.header = data.header
    #     for i in range(0,len(data.objects.data),12):
    #         try:
    #             objId = data.objects.data[i]
    #             (trans,quat) = listener.lookupTransform(data.header.frame_id, objFramePrefix_+'_'+str(int(objId)), data.header.stamp)
    #             tag = AprilTagDetection()
    #             tag.id.append(objId)
    #             tag.pose.pose.pose.position.x = trans[0]
    #             tag.pose.pose.pose.position.y = trans[1]
    #             tag.pose.pose.pose.position.z = trans[2]
    #             tag.pose.pose.pose.orientation.x = quat[0]
    #             tag.pose.pose.pose.orientation.y = quat[1]
    #             tag.pose.pose.pose.orientation.z = quat[2]
    #             tag.pose.pose.pose.orientation.w = quat[3]
    #             tag.pose.header = output.header
    #             if distanceMax_ <= 0.0 or trans[2] < distanceMax_:
    #                 output.detections.append(tag)


def py_accu_check():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('py_accu_check', anonymous=True)

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    py_accu_check()