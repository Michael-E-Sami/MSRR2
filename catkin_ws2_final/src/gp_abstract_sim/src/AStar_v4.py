#!/usr/bin/env python
"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math
import rospy
from matplotlib import pyplot as plt
from time import sleep
#from mybot_gazebo.msg import PlanarOdometry, PlanarPose, PlanarTwist
from gp_abstract_sim.msg import path_points
from nav_msgs.msg import Odometry
#from AStarPlanner import *
from tf.transformations import euler_from_quaternion
show_animation = True
plt.ion()
sx = 0.0  # [m]
sy = 0.0
syaw = 0.0
gx = 0.0  # [m]
gy = 0.0
gtheta = 0.0
robot_radius = 0.25  # [m]
rx = []
ry = []
ox, oy = [], []
flagoff=False
    
for i in range(-2, 2):
    ox.append(i)
    oy.append(-2.0)
for i in range(-2, 2):
    ox.append(2.0)
    oy.append(i)
for i in range(-2, 2):
    ox.append(i)
    oy.append(2.0)
for i in range(-2, 2):
    ox.append(-2.0)
    oy.append(i)
class AStarPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                # if len(closed_set.keys()) % 10 == 0:
                #     plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, minp):
        """
        calc grid position
        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        # print("X input range: [", self.minx, " --> ", self.maxx, "]")
        # print("Y input range: [", self.miny, " --> ", self.maxy, "]")
        # print("miny:", self.miny)
        # print("maxx:", self.maxx)
        # print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        self.xwidth = int(self.xwidth)
        self.ywidth = int(self.ywidth)

        # print("xwidth:", self.xwidth)
        # print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
path_pub = rospy.Publisher('AStarPath', path_points, queue_size=5)
#########################
def callbackG(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard ")
    global gx
    global gy
    global gtheta
    
    gx = data.pose.pose.position.x
    gy = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, gtheta) = euler_from_quaternion([rot_q.x, rot_q.y,rot_q.z, rot_q.w])

def callbackS(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard ")
    global sx
    global sy
    global syaw
    
    sx = data.pose.pose.position.x
    sy = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, syaw) = euler_from_quaternion([rot_q.x, rot_q.y,rot_q.z, rot_q.w])

###################################################################

def main(grid_size):
    print("\n \nStart!!")
    rospy.init_node('a_star_test', anonymous=True, disable_signals=True)   

    xmin = round(min(ox))
    xmax = round(max(ox))
    ymin = round(min(oy))
    ymax = round(max(oy))

    print("X input range: ["),
    print(xmin),
    print(" --> "),
    print(xmax),
    print("]")
    print("Y input range: ["),
    print(ymin),
    print(" --> "),
    print(ymax),
    print("]\n")
    dittoS = str(input("please enter the number of the robot you want to start from: "))
    dittoG = str(input("please enter the number of the robot you want to attach to: "))

    rospy.Subscriber("ditto" + dittoS +"/odom", Odometry, callbackS)
    rospy.Subscriber("ditto" + dittoG +"/odom", Odometry, callbackG)
    rate = rospy.Rate(15)
    global flagoff
    while not rospy.is_shutdown():
        if (sx == 0.0 and sy == 0.0 and syaw == 0.0):
            rospy.wait_for_message("ditto" + dittoS +"/odom", Odometry, timeout=10)
        sx_round = round(sx, 2)
        sy_round = round(sy, 2) 
        print("Current X Position: "),
        print(sx_round)
        print("Current Y Position: "),
        print(sy_round)
        print("Current Yaw: "),
        print(syaw)
        print(" ")
        if (gx == 0.0 and gy == 0.0 and gtheta == 0.0):
            rospy.wait_for_message("ditto" + dittoG +"/odom", Odometry, timeout=10)
        print ("The Goal Pose You Requested is :"),
        print(gx, gy, gtheta)
        gx_round = round(gx, 2) 
        gy_round = round(gy, 2)
        print("sx_round:: ",str(sx_round),"sy_round:: ",str(sy_round))
        print("gx_round:: ",str(gx_round),"gy_round:: ",str(gy_round))
        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.plot(sx_round, sy_round, "og")
            plt.plot(gx_round, gy_round, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.show()
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)
        rx.reverse()
        ry.reverse()
        rx_round = [round(num, 2) for num in rx]
        ry_round = [round(num, 2) for num in ry]
        print("rx: "),
        print(rx_round)
        print("ry: "),
        print(ry_round)
        path = path_points()
        path.x = rx_round
        path.y = ry_round
        path.yaw = gtheta
        path.dOne = dittoS
        path.dTwo = dittoG
        if show_animation:  # pragma: no cover
            plt.figure(num="Planned Path")
            plt.grid(True)
            plt.xlabel("X-Position")
            plt.ylabel("Y-Position")
            plt.plot(rx_round, ry_round, "-r")
            plt.autoscale(enable=True, axis='both')
            plt.show()
        #     path_pub.publish(path)
        #     flagoff==True
	    # if flagoff==True:
        # 	rate.sleep()
            #flagoff=True
        #if flagoff==True:
	       #rospy.signal_shutdown(10)
        
        answer = None
        while answer not in ("yes", "no"):
            answer = raw_input("Do you want to edit the plan?,Answer with 'yes' or 'no': ")
            if answer == "yes":
                plt.close('all')
                #print(" ")
            elif answer == "no":
                while True:
                    path_pub.publish(path)
                    rate.sleep()              
            else:
                print("Please enter 'yes' or 'no'.")    
        #rospy.spin()
if __name__ == '__main__':
    main(0.1)
