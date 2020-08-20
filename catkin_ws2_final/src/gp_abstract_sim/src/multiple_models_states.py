#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

#######################################################################################################

models_states = ModelStates()

dittos_states = ModelStates()

index = 0

#######################################################################################################


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard ")
    global models_states
    global dittos_states

    models_states = data

    new_array_size = len(models_states.name)
    # print("New Array Size = "),
    # print(new_array_size)
    # print(" ")
    global index
    # while not rospy.is_shutdown():
    if new_array_size != 0:
        for index in range(new_array_size):
            # print("Index now is: "),
            # print(index)
            # print(" ")
            if not (models_states.name[new_array_size - index - 1].startswith('ditto')):
                models_states.name.pop(new_array_size - index - 1)
                models_states.pose.pop(new_array_size - index - 1)
                models_states.twist.pop(new_array_size - index - 1)
                # index = 0
            # else:
            #     index = index + 1
            # print("Name now is like this: "),
            # print(models_states.name)
            # print(" ")
        # new_array_size = len(models_states.name)
        # print("IN LOOP Array Size = "),
        # print(new_array_size)
        # print(" ")
        # if new_array_size == 0:
        #     break

        dittos_states = models_states
    #     print("For Array Size = "),
    #     print(new_array_size)
    #     print(" ")
    # print(" ")
    # print("After Popping: ")
    # print("Name now is like this: "),
    # print(models_states.name)
    # print(" ")

#######################################################################################################


def main():
    print("All Dittos' States Acquisition.... <3")

    rospy.init_node('get_all_states', anonymous=True, disable_signals=True)

    rospy.Subscriber("gazebo/model_states", ModelStates, callback)

    rospy.wait_for_message("gazebo/model_states", ModelStates, timeout=20)

    # print("Original Names now is like this: "),
    # print(models_states.name)
    print(" ")

    while not rospy.is_shutdown():
        # rospy.spin()
        ditto_number = len(dittos_states.name)

        if ditto_number != 0:
            for j in range(ditto_number):
                ditto_case = dittos_states.name[j]
                if ditto_case == 'ditto0':
                    ditto_odom = Odometry()
                    ditto_odom.child_frame_id = dittos_states.name[j]
                    ditto_odom.pose.pose = dittos_states.pose[j]
                    ditto_odom.twist.twist = dittos_states.twist[j]
                    pub = rospy.Publisher(
                        '/ditto0/odom', Odometry, queue_size=1000)
                    pub.publish(ditto_odom)
                    print("ditto0 Caught in Scene")

                if ditto_case == 'ditto1':
                    ditto_odom = Odometry()
                    ditto_odom.child_frame_id = dittos_states.name[j]
                    ditto_odom.pose.pose = dittos_states.pose[j]
                    ditto_odom.twist.twist = dittos_states.twist[j]
                    pub = rospy.Publisher(
                        '/ditto1/odom', Odometry, queue_size=1000)
                    pub.publish(ditto_odom)
                    print("ditto1 Caught in Scene")

                elif ditto_case == 'ditto2':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto2/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto2 Caught in Scene")

                elif ditto_case == 'ditto3':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto3/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto3 Caught in Scene")

                elif ditto_case == 'ditto4':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto4/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto4 Caught in Scene")

                elif ditto_case == 'ditto5':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto5/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto5 Caught in Scene")

                elif ditto_case == 'ditto6':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto6/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto6 Caught in Scene")

                elif ditto_case == 'ditto7':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto7/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto7 Caught in Scene")

                elif ditto_case == 'ditto8':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto8/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto8 Caught in Scene")

                elif ditto_case == 'ditto9':
                    ditto_odom_1 = Odometry()
                    ditto_odom_1.child_frame_id = dittos_states.name[j]
                    ditto_odom_1.pose.pose = dittos_states.pose[j]
                    ditto_odom_1.twist.twist = dittos_states.twist[j]
                    pub1 = rospy.Publisher(
                        '/ditto9/odom', Odometry, queue_size=1000)
                    pub1.publish(ditto_odom_1)
                    print("ditto9 Caught in Scene")

            print(" ")

        else:
            print("No Ditto Models in Scene...")
            print(" ")

        # print("Popped Names now is like this: "),
        # print(dittos_states.name)
        # print(" ")

    rospy.signal_shutdown("Aho keda")

#######################################################################################################


if __name__ == '__main__':
    main()

