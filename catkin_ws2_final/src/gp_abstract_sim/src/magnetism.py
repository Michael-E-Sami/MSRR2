#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from AStar_v6 import *
from gp_abstract_sim.msg import path_points

class magnetism():

    def __init__(self):
        rospy.init_node("magnet")
        self.sub1= rospy.Subscriber("/ditto0/sensor_data",String,self.callback0,"ditto0")
        self.sub2= rospy.Subscriber("/ditto1/sensor_data",String,self.callback0,"ditto1")
        self.sub3= rospy.Subscriber("/ditto2/sensor_data",String,self.callback0,"ditto2")
        self.sub4= rospy.Subscriber("/ditto3/sensor_data",String,self.callback0,"ditto3")
        self.movingSub = rospy.Subscriber("/AStarPath", path_points,self.callback_path)
        # self.sub5= rospy.Subscriber("/ditto4/sensor_data",String,self.callback0,"ditto4")
        self.str1="hamada"
        self.pub1 =rospy.Publisher("/ditto0/attach",String, queue_size= 20)
        self.pub2 =rospy.Publisher("/ditto1/attach",String, queue_size= 20)
        self.pub3 =rospy.Publisher("/ditto2/attach",String, queue_size= 20)
        self.pub4 =rospy.Publisher("/ditto3/attach",String, queue_size= 20)
        # self.pub5 =rospy.Publisher("/ditto4/attach",String, queue_size= 20)
        rospy.spin()
    def callback_path(self,msg):
        self.movingditto = msg.dOne


    def callback0(self,msg,args):

        self.str1=msg.data
        cut = self.str1.split('#')
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
        print first[1].find('north')
        if first_pol == second_pol:
            tree= first[0] + "/"+first_body +"#" + second[0]+"/"+second_body
        else:
           
            tree= first[0] + "/"+first_body +"#" + second[0]+"/"+second_body
            # ditto1/body#ditto2/front_face
        print tree

        self.pub1.publish(tree)
        self.pub2.publish(tree)
        self.pub3.publish(tree)
        self.pub4.publish(tree)
        #self.pub5.publish(tree)




  



if __name__ == "__main__":
    hamada = magnetism()
    
#"ditto1::body_magnet_south::box_collision#ditto2::front_face_magnet_south::box_collision"



