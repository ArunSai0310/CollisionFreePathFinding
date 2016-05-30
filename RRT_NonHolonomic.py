"""
****Arun Sai Sangawar Vijay**
*********800890154***********
"""
__author__ = 'ArunSai'

import math
import random
import numpy as np
import pylab as pl
import Graph
import DiffDrive_Euler
import DiffDrive_RungeKutta
import matplotlib
import sys

from matplotlib import pyplot as plt
from matplotlib import collections as mc
from shapely.geometry import Point
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import MultiPolygon as ShapelyMultiPolygon
from shapely.geometry import LineString
from matplotlib.patches import Polygon, Rectangle, Circle
from matplotlib.collections import PatchCollection

#An instance for the graph datastructure
graph = Graph.Graph()

fig, ax = pl.subplots()
patches = []
index1 = 0
index2 = 1

# Returns the nearest point to "randpt" among the set of points in "pt"
def nearest(pt, randpt):
    i=0
    dist = []
    # randpt = np.array(randpt)
    # print("pt", pt)
    #return min(pt, key=lambda x: np.linalg.norm(np.array(list(x)) - randpt))
    while i < len(pt):
        # exp = math.sqrt((rx-arrayx[i])**2+(ry-arrayy[i])**2)
        # dist.append(exp)
        x = np.array([randpt])
        y = np.array([pt[i]])

        expr = np.linalg.norm(x - y)

        dist.append(expr)
        i += 1
    # print(dist)
    # print(dist.index(min(dist)))
    return pt[dist.index(min(dist))]


# Main part of the code, which generates and plots the RRT for Diff Drive
if __name__ == "__main__":

    # color = []
    # Selecting a default source point
    source = (90,20,math.pi/18)
    # List which stores the points generated in this process starting from the source
    pts = [source]
    # ur = velocity of right wheel, ul = velocity of left wheel set to a constant value '1' radians/second
    ur = 2
    ul = 2

    # This defines the obstacle area for Collision Detection which is done using "Shapely"
    poly = ShapelyPolygon(((5, 5), (30, 25), (38, 15), (9, 2), (5, 5)))
    poly2 = ShapelyPolygon(((40, 40), (60, 60), (50, 40), (60, 20), (40, 40)))
    poly3 = ShapelyPolygon(((38, 70), (58, 70), (58, 95), (38, 90), (38, 70)))
    poly4 = ShapelyPolygon(((5, 40), (25, 40), (25, 60), (5, 60), (5, 40)))
    poly5 = ShapelyPolygon(((70, 60), (100, 60), (100, 90), (70, 90), (70,60)))
    polygons = ShapelyMultiPolygon([poly,poly2,poly3,poly4,poly5])

    # This list is to store the points which are used in RRT generation
    lines = []
    # This list is used to store the points leading to the destination forming shortest path
    lines2 = []
    # for i in range(12):
    #     color.append((1, 0, 0, 1))

    # This iterates for a specified number of times, in each iteration performs a set of tasks to get a newpoint for building the RRT
    for i in range(10000):
        sys.stdout.write("\rCountdown: %d" % i)
        sys.stdout.flush()
        #Randomly generates a point with (x,y,theta) random values
        randompt = (random.uniform(0,100),random.uniform(0,100),random.uniform(0.0,2*math.pi))
        #Finds nearest point to the random point
        x_y = nearest(pts,randompt)

        # euler = DiffDrive_Euler.Euler(x_y,ur,ul)
        runge_kutta = DiffDrive_RungeKutta.RungeKutta(x_y,ul,ur)
        #this function call returns three possible points straight, left, right
        # possible_euler_pts = euler.euler_method()
        possible_pts = runge_kutta.runge_kutta()
        # print "straight , left , Right ",possible_euler_pts
        # finds the nearest of the three points to the random point
        newpt = nearest(possible_pts,randompt)

        #Collision check whether the newpoint generated lies inside any obstacle or not
        if polygons.contains(Point((newpt[0],newpt[1]))):
            continue
        #Check for new point not crossing the boundary of environment
        if newpt[0]>100 or newpt[0]<0 or newpt[1]<0 or newpt[1]>100:
            continue

        #generate a line from the nearest point "x-y" to the "newpt" and check whether the line is crossing any obstacle if not add line to the environment
        line = [(x_y[0],x_y[1]),(newpt[0],newpt[1])]
        if polygons.crosses(LineString(line)):
            continue
        lines.append(line)
        # color.append((0, 1, 1, 1))
        # Add the line generated as an edge to the graph from vertex at point "x_y" to "newpt"
        graph.addEdge(index1,x_y,index2,newpt)
        index1 += 2
        index2 += 2
        # Appends the newpt(new point) generated in each iteration to the "pts" (points) list.
        pts.append(newpt)

    # print(pts)


    # print graph.vertexMap
    # calls the shortest path method using the instance of the class Graph by passing source point as an argument. This finds the short routes from source to every other point in the tree
    graph.shortestPath(source)
    # Generating a new point as the goal
    # newrandompt = (random.uniform(0,100),random.uniform(0,100),random.uniform(0.0,2*math.pi))
    newrandompt = (7.0,83.7,3.14)
    #returns the nearest point to the destination in the tree
    nearestToGoal = nearest(pts,newrandompt)
    # calls the printShortPath method which returns a path from point nearest to goal to source
    path = graph.printShortPath(nearestToGoal)

    # print path,len(path),newrandompt,nearestToGoal
    # c = np.array(color)
    l=0
    # this part will generate lines for the shortest path returned
    while l<(len(path)-1):
        line = [(path[l][0],path[l][1]),(path[l+1][0],path[l+1][1])]
        lines2.append(line)
        # color.append((0, 0, 0, 1))
        l+=1
    if(l==(len(path)-1)):
        line = [(path[l][0],path[l][1]),(newrandompt[0],newrandompt[1])]
        if polygons.crosses(LineString(line)):
            print "Generated Tree can't reach the destination need more vertices."
        else:
            lines2.append(line)

        # color.append((0, 0, 0, 1))

    # print(lines)
    # print(lines2)


    # generating plotting for the lines needed to build the RRT environment
    color_lines = np.tile((0.282353, 0.819608, 0.8, 1), (len(lines), 1))
    lc = mc.LineCollection(lines, colors=color_lines, linewidths=2)
    ax.add_collection(lc)
    # generating plotting for the lines which represent path from source to destination in the environment
    color_lines2 = np.tile((0.133333, 0.545098, 0.133333, 1), (len(lines2), 1))
    lc2 = mc.LineCollection(lines2, colors=color_lines2, linewidths=2)
    ax.add_collection(lc2)

    # Plotting the obstacles to the environment
    polygon1 = Polygon(((5, 5), (30, 25), (38, 15), (9, 2), (5, 5)), closed=True, color='r')
    patches.append(polygon1)
    polygon2 = Polygon(((40, 40), (60, 60), (50, 40), (60, 20), (40, 40)), closed=True, color='r')
    patches.append(polygon2)
    polygon3 = Polygon(((38, 70), (58, 70), (58, 95), (38, 90), (38, 70)), closed=True, color='r')
    patches.append(polygon3)
    polygon4 = Polygon(((5, 40), (25, 40), (25, 60), (5, 60), (5, 40)), closed=True, color='r')
    patches.append(polygon4)
    polygon5 = Polygon(((70, 60), (100, 60), (100, 90), (70, 90), (70,60)), closed=True, color='r')
    patches.append(polygon5)
    square = Rectangle((source[0], source[1]), 2, 2, facecolor="grey")
    patches.append(square)
    circle = Circle((7.0,83.7), 1, ec="none")
    patches.append(circle)

    # colors = 100*np.random.rand(len(patches))
    # colors = 100*np.array((0.69,0.65,0.69))
    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)
    # p.set_array(np.array(colors))
    ax.add_collection(p)

    plt.xlim([-2, 100])
    plt.ylim([-2, 100])

    plt.show()






