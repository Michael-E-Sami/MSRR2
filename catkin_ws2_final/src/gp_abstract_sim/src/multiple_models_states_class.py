#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

#######################################################################################################



#######################################################################################################
class multipleModelStates():


    def callback(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard ")
        models_states = data

        new_array_size = len(models_states.name)

        global index

        if new_array_size != 0:
            for index in range(new_array_size):

                if not (models_states.name[new_array_size - index - 1].startswith('ditto')):
                    models_states.name.pop(new_array_size - index - 1)
                    models_states.pose.pop(new_array_size - index - 1)
                    models_states.twist.pop(new_array_size - index - 1)


            self.dittos_states = models_states

        self.waitForGo = False


    #######################################################################################################


    def __init__(self):
        print("All Dittos' States Acquisition.... <3")
        if isMain:
            rospy.init_node('get_all_states', anonymous=True, disable_signals=True)

        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback)

        rospy.wait_for_message("gazebo/model_states", ModelStates, timeout=20)

        pub = {}

        for i in range(len(self.dittos_states.name)):
            pub[self.dittos_states.name[i]] = rospy.Publisher(self.dittos_states.name[i] + "/odom", Odometry, queue_size=1000)

        self.waitForGo = True
        while self.waitForGo:
            pass
        # self.models_states = ModelStates()

        self.dittos_states = ModelStates()

        # print("Original Names now is like this: "),
        # print(models_states.name)
        print(" ")

        while not rospy.is_shutdown():
            # rospy.spin()
            ditto_number = len(self.dittos_states.name)

            if ditto_number != 0:
                for i in range(ditto_number):
                    ditto_odom = Odometry()
                    ditto_odom.child_frame_id = self.dittos_states.name[i]
                    ditto_odom.pose.pose = self.dittos_states.pose[i]
                    ditto_odom.twist.twist = self.dittos_states.twist[i]
                    pub[self.dittos_states.name[i]].publish(ditto_odom)
                    print(ditto_odom.child_frame_id + " Caught in Scene")
                # for j in range(ditto_number):
                #     ditto_case = self.dittos_states.name[j]
                #
                #     if ditto_case == 'my_robot':
                #         my_robot_odom = Odometry()
                #         my_robot_odom.child_frame_id = self.dittos_states.name[j]
                #         my_robot_odom.pose.pose = self.dittos_states.pose[j]
                #         my_robot_odom.twist.twist = self.dittos_states.twist[j]
                #         pub = rospy.Publisher(
                #             '/my_mine_robot/odom', Odometry, queue_size=1000)
                #         pub.publish(my_robot_odom)
                #         print("my_robot Caught in Scene")
                #
                #     elif ditto_case == 'my_robot_1':
                #         my_robot_odom_1 = Odometry()
                #         my_robot_odom_1.child_frame_id = self.dittos_states.name[j]
                #         my_robot_odom_1.pose.pose = self.dittos_states.pose[j]
                #         my_robot_odom_1.twist.twist = self.dittos_states.twist[j]
                #         pub1 = rospy.Publisher(
                #             '/my_mine_robot_1/odom', Odometry, queue_size=1000)
                #         pub1.publish(my_robot_odom_1)
                #         print("my_robot_1 Caught in Scene")

                print(" ")

            else:
                print("No Ditto Models in Scene...")
                print(" ")



        rospy.signal_shutdown("Aho keda")

#######################################################################################################

isMain = False
if __name__ == '__main__':
    isMain = True
    start = multipleModelStates()
