# -*- coding: utf-8 -*-
"""FMT.ipynb

Original file is located at
    https://colab.research.google.com/drive/15UZySk2iueEvuoWKXLZLdXWhHYNCyza3
"""
import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import matplotlib.patches as patches
from scipy import interpolate
# from Availibity_checker import CheckObstacleCollision 

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.cost = np.inf

class FMT:
    def __init__(self, x_start, x_goal, search_radius,dominion,sample_numbers,state_validity_checker):
        
        print('x_start ',x_start)
        print('x_goal ',x_goal)
        self.x_init = Node(x_start)
        self.x_goal = Node(x_goal)
        self.search_radius = search_radius
        # self.map= map_matrix

        self.svc = state_validity_checker

        # self.fig, self.ax = plt.subplots()
        self.delta = 1
        self.x_range = (dominion[0],dominion[1])
        self.y_range = (dominion[0],dominion[1])
        print('x_range: ', self.x_range)
        print('y_range: ', self.y_range)
        self.V = set()
        self.V_unvisited = set()
        self.V_open = set()
        self.V_closed = set()
        self.sample_numbers = sample_numbers

    def Init(self):
        samples = self.SampleFree()
        self.x_init.cost = 0.0
        self.V.add(self.x_init)
        self.V.update(samples)
        self.V_unvisited.update(samples)
        self.V_unvisited.add(self.x_goal)
        self.V_open.add(self.x_init)

    def Planning(self):
        self.Init()
        z = self.x_init
        n = self.sample_numbers
        rn = self.search_radius * math.sqrt((math.log(n) / n))
        Visited = []

        # keep track of visited nodes and edges
        visited_nodes = set()
        visited_edges = set()

        while z is not self.x_goal:
            V_open_new = set()
            X_near = self.Near(self.V_unvisited, z, rn)
            Visited.append(z)

            for x in X_near:
                Y_near = self.Near(self.V_open, x, rn)
                cost_list = {y: y.cost + self.Cost(y, x) for y in Y_near}
                y_min = min(cost_list, key=cost_list.get)

                if not self.svc.CheckObstacleCollision((y_min.x, y_min.y), (x.x, x.y)):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.Cost(y_min, x)

                    # add visited edges
                    visited_edges.add(((y_min.x, y_min.y), (x.x, x.y)))

            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.V_closed.add(z)
            visited_nodes.add((z.x, z.y))

            if not self.V_open:
                print("open set empty!")
                break

            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)

        # add visited nodes for the goal node
        visited_nodes.add((self.x_goal.x, self.x_goal.y))

        # plot the visited nodes and edges
        # for node in visited_nodes:
        #     self.ax.plot(node[0], node[1], 'bo')
        # for edge in visited_edges:
        #     self.ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], 'b')

        path_x, path_y = self.ExtractPath()

        return (path_x, path_y)


    def ChooseGoalPoint(self):
        Near = self.Near(self.V, self.x_goal, 2.0)
        cost = {y: y.cost + self.Cost(y, self.x_goal) for y in Near}
        return min(cost, key=cost.get)

    def ExtractPath(self):
        path_x, path_y = [], []
        node = self.x_goal
        while node.parent:
            path_x.append(node.x)
            path_y.append(node.y)
            node = node.parent
        path_x.append(self.x_init.x)
        path_y.append(self.x_init.y)
        return path_x, path_y

    def Cost(self, x_start, x_end):
        if self.svc.CheckObstacleCollision((x_start.x, x_start.y), (x_end.x, x_end.y)): 
            return np.inf
        else:
            return self.calc_dist(x_start, x_end)

    def calc_dist(self,x_start, x_end):
        return math.hypot(x_start.x - x_end.x, x_start.y - x_end.y)

    def Near(self,nodelist, z, rn):
        return {nd for nd in nodelist
                if 0 < (nd.x - z.x) ** 2 + (nd.y - z.y) ** 2 <= rn ** 2}

    def SampleFree(self):
        n = self.sample_numbers
        delta = self.delta
        Sample = set()

        ind = 0
        while ind < n:
            node = Node((random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
            Sample.add(node)
            ind += 1

        return Sample

def main():
    map_matrix = np.array(np.loadtxt("maps/np_map2_dilated.txt"))
    x_start = (140, 10)  # Starting node
    x_goal = (140, 60)  # Goal node
    sample_numbers = 100
    search_radius = 400
    path=[]
    fmt = FMT(x_start, x_goal, search_radius,map_matrix,sample_numbers)
    path_x,path_y = fmt.Planning()
if __name__ == '__main__':
    main()