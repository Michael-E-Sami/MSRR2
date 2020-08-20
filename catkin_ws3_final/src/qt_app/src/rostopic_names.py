#!/usr/bin/env python
import rospy
import rostopic
import PyQt5
# print(rospy.get_published_topics())
# tpc = rostopic.get_topic_list()
# print(tpc[1][1][1])
# print(tpc[0])
# print(len(tpc))
# print(len(tpc[1]))
# print(len(tpc[1][7]))
# print(rostopic.get_topic_list())
tpc = rostopic.get_topic_list()
my_list = []
for x in range(len(tpc)):
    for y in range(len(tpc[x])):
        # for z in range(len(tpc[x][y])):
        for z in range(2):
            # print(tpc[x][y][z])
            # print(tpc[x][y][z].find('ditto'))
            if tpc[x][y][z].find('ditto') >= 0:
                # print(tpc[x][y][z])
                string_smth = tpc[x][y][z].partition('/')[2]
                to_go_to_list = string_smth.partition('/')[0]
                # print(to_go_to_list)
                if not to_go_to_list in my_list:
                    my_list.append(to_go_to_list)


arranged_list = []
for x in my_list:
    arranged_list.append(int(x.lstrip('ditto')))

arranged_list.sort()
final_list = []
for x in arranged_list:
    final_list.append('ditto' + str(x))
# print(arranged_list)
# print(final_list)
# print(my_list)
# print(my_list[0].lstrip('ditto'))
