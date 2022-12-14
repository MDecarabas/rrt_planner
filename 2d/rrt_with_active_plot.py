#!/usr/bin/env python

from __future__ import print_function
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pprint

def calc_distance(p_1, p_2):
    """
    Calculates distance between two points.
    ARGUMENTS
        p_1         Point 1 coordinates; tuple
        p_2         Point 2 coordinates; tuple
    OUTPUT
        distance    Straight-line distance from p_1 -> p_2; scalar
    """
    distance = math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)
    return distance

def obstacle_check(p_check):
    """
    Checks whether a point is inside any of the obstacles.
    ARGUMENTS
        p_check         Point to check; tuple
    OUTPUT
        in_obstacles    Obstacle collision conditions; list of Boolean values
    """
    in_obstacles = []                   # intersection condition (Boolean) with each obstacle
    for ind_a in range(N):              # for each obstacle
        # calculate distance from point to obstacle center
        distance_to = calc_distance(OBSTACLE_PROPS[ind_a][0], p_check)
        # check distance against obstacle radius
        if distance_to <= OBSTACLE_PROPS[ind_a][1]:     # if radius > distance
            in_obstacles.append(True)                   # mark point within obstacle
        else:                                           # if distance > radius
            in_obstacles.append(False)                  # mark point outside obstacle

    return in_obstacles

def point2line_params(p_1, p_2, p_3):
    """
    Defines a line passing through two points. Determines whether the tangent to
    the line passing through a third point intersects between the first two. Formula is
    defined at http://paulbourke.net/geometry/pointlineplane/
    ARGUMENTS
        p_1     Point 1 coordinates; tuple
        p_2     Point 2 coordinates; tuple
        p_3     Point 3 coordinates; tuple
    OUTPUT
    """
    seg_vec = (p_2[0] - p_1[0], p_2[1] - p_1[1])                # P1 -> P2 vector
    seg_mag = math.sqrt(seg_vec[0]**2 + seg_vec[1]**2)          # length of P1 -> P2 segment

    # determine intersection length ratio u
    u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(seg_mag**2)

    # coordinates of intersection point
    p_x = p_1[0] + u*(p_2[0] - p_1[0])
    p_y = p_1[1] + u*(p_2[1] - p_1[1])

    # distance from P3 to intersection point
    distance = calc_distance((p_x, p_y), p_3)

    return u, distance

# generate obstacles
N = 20                              # number of obstacles
OBSTACLE_PROPS = []                 # list to hold obstacle centers/radii
# x- and y-coordinates of obstacles
OBSTACLES_X = 100*np.random.rand(N)
OBSTACLES_Y = 100*np.random.rand(N)
# list of obstacle center coordinates as tuples
OBSTACLE_CENTERS = list(zip(OBSTACLES_X, OBSTACLES_Y))
# obstacle radii
OBSTACLE_RADII = 10*np.random.rand(N)
# fill obstacle properties list
for i in range(N):
    props = [OBSTACLE_CENTERS[i], OBSTACLE_RADII[i]]
    OBSTACLE_PROPS.append(props)

# create start point
START_OK = False                                # condition for correctly-generated start point
while not START_OK:
    START_X = 10 + 20*np.random.rand()          # x-coordinate
    START_Y = 10 + 20*np.random.rand()          # y-coordinate
    START = (START_X, START_Y)                  # start point coordinates

    START_COLLISIONS = obstacle_check(START)    # check for collision between
                                                # start point and obstacles

    if any(START_COLLISIONS):                   # if a collision with any obstacle exists
        pass                                    # do nothing; loop will re-run
    else:                                       # if no collision exists
        START_OK = True                         # clear to move forward
START_NAME = 'q0'                               # start point node name
POINTS = [[START_NAME, START, 'None']]          # initialize point data list

# create end point
END_OK = False                                  # condition for correctly-generated end point
while not END_OK:
    END_X = 70 + 20*np.random.rand()            # x-coordinate
    END_Y = 70 + 20*np.random.rand()            # y-coordinate
    END = (END_X, END_Y)

    END_COLLISIONS = obstacle_check(END)        # check for collisions between
                                                # end point and obstacles

    if any(END_COLLISIONS):                     # if any collision exists
        pass                                    # do nothing, loop will re-run
    else:                                       # if no collision
        END_OK = True                           # move forward

SEGMENTS = []                       # empty list to hold path segments
CLEAR_PATH = False                  # "clear path to end point exists" condition
while not CLEAR_PATH:
    POINT_OK = False                # condition for correctly-generated new node
    # generate new node and check for collisions with obstacles
    while not POINT_OK:
        POINT_NAME = "q{}".format(len(POINTS))
        # coordinates of random point
        POINT_X = np.random.randint(0, 101)
        POINT_Y = np.random.randint(0, 101)

        # determine node closest to random point, distance to it, and its list index
        MIN_DIST = 10e10                            # initial distance to closest node
        for i in range(len(POINTS)):                # iterate through existing nodes
            DIST_TO_POINT = calc_distance(POINTS[i][1], (POINT_X, POINT_Y))     # calculate distance to each node
            if DIST_TO_POINT <= MIN_DIST:           # if distance to node is less than minimum distance
                MIN_DIST = DIST_TO_POINT            # save distance to node as minimum distance
                CLOSEST_NODE = i                    # save list index of closest node

        PARENT = "q{}".format(CLOSEST_NODE)         # "parent" node name
        PARENT_COORDS = (POINTS[CLOSEST_NODE][1])   # "parent" node coordinates

        # create vector from closest node to random point
        D_X = POINT_X - POINTS[CLOSEST_NODE][1][0]
        D_Y = POINT_Y - POINTS[CLOSEST_NODE][1][1]
        SHORTEST_VECTOR = [D_X, D_Y]
        # magnitude of vector to closest node
        SHORTEST_MAG = math.sqrt((D_X**2)+(D_Y**2))
        # calculate unit vector
        UNIT_VECTOR = [dist/SHORTEST_MAG for dist in SHORTEST_VECTOR]

        # coordinates of new node
        NODE_X = POINTS[CLOSEST_NODE][1][0] + UNIT_VECTOR[0]
        NODE_Y = POINTS[CLOSEST_NODE][1][1] + UNIT_VECTOR[1]

        # check for collisions with obstacles
        NODE_COLLISIONS = obstacle_check((NODE_X, NODE_Y))

        if any(NODE_COLLISIONS):                    # if collision exists
            pass                                    # do nothing and re-run loop
        else:                                       # if no collisions
            POINT_OK = True                         # move forward

    # add new node data to list
    POINTS.append([POINT_NAME, (NODE_X, NODE_Y), PARENT])

    # add new path segment to list
    SEGMENTS.append([PARENT_COORDS, (NODE_X, NODE_Y)])

    # determine if line to endpoint intersects obstacles
    LINE_COLLISIONS = []        # list of obstacle collision conditions
    for j in range(N):
        TOO_CLOSE = False       # obstacle intersection condition
        BETWEEN = False         # intersection between endpoints condition

        # get intersection length ratio and distance to obstacle center
        RATIO_U, DIST_TO_OBSTACLE = point2line_params(POINTS[-1][1], END, OBSTACLE_PROPS[j][0])

        # intersection with obstacle is between node and endpoint if 0 < u < 1
        if 0 <= RATIO_U <= 1:
            BETWEEN = True

        # line intersects obstacle if distance to obstacle center < obstacle radius
        if DIST_TO_OBSTACLE <= OBSTACLE_PROPS[j][1]:
            TOO_CLOSE = True

        # if line to endpoint intersects obstacle between node and endpoint, path is blocked
        if BETWEEN and TOO_CLOSE:
            LINE_COLLISIONS.append(True)        # add obstacle collision condition to list
        else:
            LINE_COLLISIONS.append(False)

    if any(LINE_COLLISIONS):                    # if any obstacle collisions exist
        pass                                    # re-run loop to generate another node
    else:                                       # if no collisions exist
        CLEAR_PATH = True                       # move forward

# generate final path segment & add to list
SEGMENTS.append([POINTS[-1][1], END])

# work backwards to identify successful path
PATH_POINTS = [POINTS[-1][1]]                   # nodes in successful path
PATH_SEGS = [[POINTS[-1][1], END]]              # start path by adding final segment
AT_START = False                                # condition for reaching start point

CURRENT_COORDS = POINTS[-1][1]                  # x-y coordinates of last point in path
PARENT_NODE = POINTS[-1][2]                     # name of initial parent node

while not AT_START:                                                 # iterate until start is reached
    for l in range(len(POINTS)):                                    # scan all nodes to find parent
        if POINTS[l][0] == PARENT_NODE:                             # if node name matches parent node name
            PATH_POINTS.insert(0, POINTS[l][1])                     # add node to path
            PATH_SEGS.insert(0, [POINTS[l][1], CURRENT_COORDS])     # add path segment from node to parent
            CURRENT_COORDS = POINTS[l][1]                           # parent node becomes new current node
            PARENT_NODE = POINTS[l][2]                              # new parent node
    if PARENT_NODE == 'None':                                       # once start point is reached
        AT_START = True                                             # move forward

# create node plot points
NODES_PLOT_X = [POINTS[a][1][0] for a in range(len(POINTS))]
NODES_PLOT_Y = [POINTS[a][1][1] for a in range(len(POINTS))]
# create path plot points
PATH_PLOT_X = [PATH_POINTS[b][0] for b in range(len(PATH_POINTS))]
PATH_PLOT_Y = [PATH_POINTS[b][1] for b in range(len(PATH_POINTS))]

# plot colors
START_COLOR = '#5dc91f'             # start point color
END_COLOR = '#ff9b00'               # end point color
POINTS_COLOR = '#1691e5'            # color for all nodes/edges
PATH_COLOR = '#c0120a'              # color of path nodes and segments

# create image
FIG, AX = plt.subplots(num = "Pathfinding RRT", nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
# axis limits
plt.xlim(0, 100)
plt.ylim(0, 100)

# plot obstacles as circular patch collection
OBSTACLES = [plt.Circle(center, radius) for center, radius in zip(OBSTACLE_CENTERS, OBSTACLE_RADII)]
PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
AX.add_collection(PATCHES)
# plot start & end points
plt.scatter(START[0], START[1], s=100, c=START_COLOR, marker='+')
plt.scatter(END[0], END[1], s=100, c=END_COLOR, marker='+')

# plot all nodes/edges one by one
for i in range(len(POINTS)):
    # plot newest node
    plt.scatter(NODES_PLOT_X[i], NODES_PLOT_Y[i], s=10, c=POINTS_COLOR)
    if i > 0:
        # plot newest edge
        NODE_SEGMENTS = mpl.collections.LineCollection(SEGMENTS[i-1:i], colors=POINTS_COLOR)
        AX.add_collection(NODE_SEGMENTS)
    plt.pause(0.1)

# plot final segment
NODE_SEGMENTS = mpl.collections.LineCollection([SEGMENTS[-1], [END]], colors=POINTS_COLOR)
AX.add_collection(NODE_SEGMENTS)
plt.pause(0.1)

for i in range(len(PATH_POINTS)):
    # plot newest path node
    plt.scatter(PATH_PLOT_X[i], PATH_PLOT_Y[i], s=10, c=PATH_COLOR)
    if i > 0:
        # plot newest edge
        PATH_SEGMENTS = mpl.collections.LineCollection(PATH_SEGS[i-1:i], colors=PATH_COLOR)
        AX.add_collection(PATH_SEGMENTS)
    plt.pause(0.05)

# plot final path segment
PATH_SEGMENTS = mpl.collections.LineCollection([PATH_SEGS[-1], [END]], colors=PATH_COLOR)
AX.add_collection(PATH_SEGMENTS)
plt.pause(0.05)

plt.show()              # show image