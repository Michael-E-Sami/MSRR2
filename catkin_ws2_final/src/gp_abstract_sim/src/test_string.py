#!/usr/bin/env python
import rospy
from std_msgs.msg import String


msg = "ditto1::body_magnet_south::box_collision#ditto2::front_face_magnet_south::box_collision"
cut = msg.split('#')
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
    print first[0] + "/"+first_body +"#" + second[0]+"/"+second_body
else:
    print 'yuppies!'
    print first[0] + "/"+first_body +"#" + second[0]+"/"+second_body



