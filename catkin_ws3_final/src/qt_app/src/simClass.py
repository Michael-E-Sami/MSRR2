from abstractClass import abstractClass
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetJointProperties
# import math
import numpy as np
class simClass(abstractClass):
    def __init__(self,robots_connected):
        self.pubDict = {}
        self.subDict = {}
        self.data = {}
        self.odom = {}
        self.direction = 1
        for i in range(len(robots_connected)):
            dittoName = "ditto" + str(robots_connected[i])
            # pub = rospy.Publisher(dittoName + "/cmd_vel", Twist, queue_size=10)
            # self.pubDict[dittoName + "/cmd_vel"] = pub
            pub = rospy.Publisher(dittoName + "/left_wheel_speed", Float32, queue_size=10)
            self.pubDict[dittoName + "/left_wheel_speed"] = pub
            pub = rospy.Publisher(dittoName + "/right_wheel_speed", Float32, queue_size=10)
            self.pubDict[dittoName + "/right_wheel_speed"] = pub
            pub = rospy.Publisher(dittoName + "/front_face", Float32, queue_size=10)
            self.pubDict[dittoName + "/front_face"] = pub
            pub = rospy.Publisher(dittoName + "/face_pan", Float32, queue_size=10)
            self.pubDict[dittoName + "/face_pan"] = pub
            sub = rospy.Subscriber(dittoName + "/body_ir", Range, self.body_ir_msg,dittoName)
            self.subDict[dittoName + "/body_ir"] = sub
            sub = rospy.Subscriber(dittoName + "/body_tof1", Range, self.body_tof1_msg,
                                   dittoName)
            self.subDict[dittoName + "/body_tof1"] = sub
            sub = rospy.Subscriber(dittoName + "/body_tof2", Range, self.body_tof2_msg,
                                   dittoName)
            self.subDict[dittoName + "/body_tof2"] = sub
            sub = rospy.Subscriber(dittoName + "/front_face_ir", Range, self.front_face_ir_msg, dittoName)
            self.subDict[dittoName + "/front_face_ir"] = sub
            sub = rospy.Subscriber(dittoName + "/front_face_tof1", Range, self.front_face_tof1_msg,
                                   dittoName)
            self.subDict[dittoName + "/front_face_tof1"] = sub
            sub = rospy.Subscriber(dittoName + "/front_face_tof2", Range, self.front_face_tof2_msg,
                                   dittoName)
            self.subDict[dittoName + "/front_face_tof2"] = sub
            sub = rospy.Subscriber(dittoName + "/odom", Odometry, self.robotOdom, dittoName)
            self.subDict[dittoName + "/robotOdom"] = sub

            self.model_coordinates = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)

        print(self.pubDict)

    def getFaceRoll(self,name):
        resp = self.model_coordinates(name + "::pan_joint")
        return resp.position[0] / np.pi * 180


    def getFacePitch(self,name):
        resp = self.model_coordinates(name + "::tilt_joint")
        return resp.position[0] / np.pi * 180


    def robotOdom(self,odom,args):
        self.odom[args] = odom

        # print(args + " linear " + str(self.odom[args].twist.twist.linear.x))
        # print(args + " angular" + str(self.odom[args].twist.twist.angular.z))

    def getPointX(self,name):
        x = self.odom[name].pose.pose.position.x
        return x

    def getPointY(self,name):
        y = self.odom[name].pose.pose.position.y
        return y

    def getRotationYaw(self,name):
        q = self.odom[name].pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def getLinearSpeed(self,name):
        return (self.odom[name].twist.twist.linear.x**2+self.odom[name].twist.twist.linear.y**2)**0.5 * self.direction

    def getAngularSpeed(self, name):
        return self.odom[name].twist.twist.angular.z

    def body_ir_msg(self,rng,args):
        name = args
        self.data[name + "/body_ir"] = rng.range
        # print("body_ir_msg" + name + ' ' + str(rng.range))

    def body_tof1_msg(self,rng,args):
        name = args
        self.data[name + "/body_tof1"] = rng.range
        # print("body_tof1_msg" + name + ' ' + str(rng.range))

    def body_tof2_msg(self,rng,args):
        name = args
        self.data[name + "/body_tof2"] = rng.range
        # print("body_tof2_msg" + name + ' ' + str(rng.range))

    def front_face_ir_msg(self,rng,args):
        name = args
        self.data[name + "/front_face_ir"] = rng.range
        # print("front_face_ir_msg" + name + ' ' + str(rng.range))

    def front_face_tof1_msg(self,rng,args):
        name = args
        self.data[name + "/front_face_tof1"] = rng.range
        # print("front_face_tof1_msg" + name + ' ' + str(rng.range))

    def front_face_tof2_msg(self,rng,args):
        name = args
        self.data[name + "/front_face_tof2"] = rng.range
        # print("front_face_tof2_msg" + name + ' ' + str(rng.range))

    def rotateLeft(self,robot_to_operate,speed):
        # twst = Twist()
        # twst.linear.x = 0
        # twst.linear.y = 0
        # twst.linear.z = 0
        # twst.angular.x = 0
        # twst.angular.y = 0
        # twst.angular.z = -speed
        # for i in robot_to_operate:
        #     self.pubDict[i + "/cmd_vel"].publish(twst)
        for i in robot_to_operate:
            self.pubDict[i + "/left_wheel_speed"].publish(-speed/0.0857*0.1)
            self.pubDict[i + "/right_wheel_speed"].publish(speed/0.0857*0.1)


    def stopRot(self,robot_to_operate):
        # twst = Twist()
        # twst.linear.x = 0
        # twst.linear.y = 0
        # twst.linear.z = 0
        # twst.angular.x = 0
        # twst.angular.y = 0
        # twst.angular.z = 0
        # for i in robot_to_operate:
        #     self.pubDict[i + "/cmd_vel"].publish(twst)
        for i in robot_to_operate:
            self.pubDict[i + "/left_wheel_speed"].publish(0.0)
            self.pubDict[i + "/right_wheel_speed"].publish(0.0)

    def rotateRight(self,robot_to_operate,speed):
        # twst = Twist()
        # twst.linear.x = 0
        # twst.linear.y = 0
        # twst.linear.z = 0
        # twst.angular.x = 0
        # twst.angular.y = 0
        # twst.angular.z = speed
        # for i in robot_to_operate:
        #     self.pubDict[i + "/cmd_vel"].publish(twst)
        for i in robot_to_operate:
            self.pubDict[i + "/left_wheel_speed"].publish(speed/0.0857*0.1)
            self.pubDict[i + "/right_wheel_speed"].publish(-speed/0.0857*0.1)

    def moveForward(self,robot_to_operate,speed):
        # twst = Twist()
        # twst.linear.x = speed
        # twst.linear.y = 0
        # twst.linear.z = 0
        # twst.angular.x = 0
        # twst.angular.y = 0
        # twst.angular.z = 0
        # for i in robot_to_operate:
        #     self.pubDict[i + "/cmd_vel"].publish(twst)
        self.direction = 1
        for i in robot_to_operate:
            self.pubDict[i + "/left_wheel_speed"].publish(speed/0.06)
            self.pubDict[i + "/right_wheel_speed"].publish(speed/0.06)

    def stopFwdBwd(self,robot_to_operate):
        # twst = Twist()
        # twst.linear.x = 0
        # twst.linear.y = 0
        # twst.linear.z = 0
        # twst.angular.x = 0
        # twst.angular.y = 0
        # twst.angular.z = 0
        # for i in robot_to_operate:
        #     self.pubDict[i + "/cmd_vel"].publish(twst)
        for i in robot_to_operate:
            self.pubDict[i + "/left_wheel_speed"].publish(0.0)
            self.pubDict[i + "/right_wheel_speed"].publish(0.0)

    def moveBackward(self,robot_to_operate,speed):
        # twst = Twist()
        # twst.linear.x = -speed
        # twst.linear.y = 0
        # twst.linear.z = 0
        # twst.angular.x = 0
        # twst.angular.y = 0
        # twst.angular.z = 0
        # for i in robot_to_operate:
        #     self.pubDict[i + "/cmd_vel"].publish(twst)
        self.direction = -1
        for i in robot_to_operate:
            self.pubDict[i + "/left_wheel_speed"].publish(-speed/0.06)
            self.pubDict[i + "/right_wheel_speed"].publish(-speed/0.06)

    def pitchUp(self,robot_to_operate,speed):
        for i in robot_to_operate:
            self.pubDict[i + "/front_face"].publish(speed)

    def stopPitch(self,robot_to_operate):
        for i in robot_to_operate:
            self.pubDict[i + "/front_face"].publish(0)

    def pitchDown(self,robot_to_operate,speed):
        for i in robot_to_operate:
            self.pubDict[i + "/front_face"].publish(-speed)

    def rollLeft(self,robot_to_operate,speed):
        for i in robot_to_operate:
            self.pubDict[i + "/face_pan"].publish(speed)

    def stopRoll(self,robot_to_operate):
        for i in robot_to_operate:
            self.pubDict[i + "/face_pan"].publish(0)

    def rollRight(self,robot_to_operate,speed):
        for i in robot_to_operate:
            self.pubDict[i + "/face_pan"].publish(-speed)


