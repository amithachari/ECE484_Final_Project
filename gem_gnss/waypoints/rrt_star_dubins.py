"""
Path planning Sample Code with RRT and Dubins path

author: AtsushiSakai(@Atsushi_twi)

"""

import copy
import math
import os
import random
import sys
import csv
import matplotlib.pyplot as plt
import numpy as np


try:
    import dubins_path_planning
    from rrt_star import RRTStar
except ImportError:
    raise

show_animation = True


class RRTStarDubins(RRTStar):
    """
    Class for RRT star planning with Dubins path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 goal_sample_rate=10,
                 max_iter=100,
                 connect_circle_dist=50.0,
                 robot_radius=3.0,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist

        # self.curvature = 1.0  # for dubins path
        self.curvature = 1/3.0
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5
        self.robot_radius = robot_radius

    def planning(self, animation=True, search_until_max_iter=True):
        """
        RRT Star planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)

            if animation and i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        # plt.axis([-20, 50, -15, 5])
        plt.axis([-45, 35, -15, 10])
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):
        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = \
            dubins_path_planning.dubins_path_planning(
                from_node.x, from_node.y, from_node.yaw,
                to_node.x, to_node.y, to_node.yaw, self.curvature)

        if len(px) <= 1:  # cannot find a dubins path
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(c) for c in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = dubins_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        cost = sum([abs(c) for c in course_lengths])

        return from_node.cost + cost

    def get_random_node(self):

        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(-39.5, 30.5),
                            random.uniform(-7.0, 2.0),
                            # random.uniform(-39.5, 30.5),
                            # random.uniform(-9.0, -6.0),
                            random.uniform(-math.pi, math.pi)
                            )
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.yaw)

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        print("final")
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])

        return path

def y2h(yaw):
    if (yaw >= -np.pi and yaw < -np.pi/2):
        heading = 90 - math.degrees(yaw)
    elif (yaw >= -np.pi/2 and yaw < 0):
        heading = 90 - math.degrees(yaw)
    if (yaw >= 0 and yaw < np.pi/2):
        heading = 90 - math.degrees(yaw)
    elif (yaw >= np.pi/2 and yaw < np.pi):
        heading = 450 - math.degrees(yaw)
    return heading
        
def main():
    print("Start rrt star with dubins planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5.0, 5.0, 0.5),
        (10.0, 5.0, 1),
        # (6.0, -2.5, 0.5),
        # (-20, -7, 2),
    ]  # [x,y,size(radius)]
    # for i in range(-15, 45, 3):
    #     obstacleList.append((i,2,0.3))
    #     obstacleList.append((i,-13,0.3))
    # for i in range(-13, 2, 1):
    #     obstacleList.append((-15,i,0.3))
    #     obstacleList.append((45,i,0.3))
        
    # print(obstacleList)
    # Set Initial parameters
    goal = [-0.6, -0.6, np.deg2rad(0.0)]
    start = [14.86, -0.7, np.deg2rad(0.0)]

    rrtstar_dubins = RRTStarDubins(start, goal, max_iter=500, rand_area=[40.0, -10.0], obstacle_list=obstacleList)
    path = rrtstar_dubins.planning(animation=show_animation)
    print(path)
    # Draw final path
    if show_animation:  # pragma: no cover
        rrtstar_dubins.draw_graph()
        plt.plot([x for (x, y, z) in path], [y for (x, y, z) in path], '-r')
        plt.grid(True)
        # plt.pause(0.001)

        plt.show()

    path.reverse()
    with open('xy_demo.csv', 'w') as file:
        writer = csv.writer(file, delimiter = '\t')
        for x,y,yaw in path:
            # writer.writerow([x, y, math.degrees(yaw)])
            file.write('%f,%f,%f\n'%(x,y,y2h(yaw)))

    plt.scatter(path[:,0],path[:,1])
    plt.show()
if __name__ == '__main__':
    main()
