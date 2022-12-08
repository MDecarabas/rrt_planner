import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def draw_obstacles(obstacles):
    for obs in obstacles:
        plt.plot(obs[0], obs[1], 'k')

def draw_path(path):
    for i in range(len(path) - 1):
        plt.plot([path[i].x, path[i+1].x], [path[i].y, path[i+1].y], 'r')

def draw_tree(tree):
    for node in tree:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'g')

def get_random_point(x_min, x_max, y_min, y_max):
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    return Node(x, y)

def get_nearest_node(tree, point):
    min_dist = float('inf')
    min_node = None
    for node in tree:
        dist = math.sqrt((node.x - point.x)**2 + (node.y - point.y)**2)
        if dist <= min_dist:
            min_dist = dist
            min_node = node
    return min_node

def get_new_point(nearest_node, point, step_size):
    dist = math.sqrt((nearest_node.x - point.x)**2 + (nearest_node.y - point.y)**2)
    if dist <= step_size:
        return point
    else:
        theta = math.atan2(point.y - nearest_node.y, point.x - nearest_node.x)
        new_x = nearest_node.x + step_size * math.cos(theta)
        new_y = nearest_node.y + step_size * math.sin(theta)
        return Node(new_x, new_y)

def check_collision(point, obstacles):
    for obs in obstacles:
        if point.x >= obs[0][0] and point.x <= obs[1][0] and point.y >= obs[0][1] and point.y <= obs[1][1]:
            return True
    return False

def get_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append(node)
        node = node.parent
    return path

def rrt(start, goal, obstacles, x_min, x_max, y_min, y_max, step_size, max_iter):
    tree = []
    tree.append(start)
    for i in range(max_iter):
        point = get_random_point(x_min, x_max, y_min, y_max)
        nearest_node = get_nearest_node(tree, point)
        new_point = get_new_point(nearest_node, point, step_size)
        if not check_collision(new_point, obstacles):
            new_point.parent = nearest_node
            tree.append(new_point)
            if math.sqrt((new_point.x - goal.x)**2 + (new_point.y - goal.y)**2) <= step_size:
                goal.parent = new_point
                tree.append(goal)
                return get_path(goal)
    return None

def main():
    start = Node(0, 0)
    goal = Node(10, 10)
    obstacles = [[(2, 2), (2, 8)], [(2, 8), (8, 8)], [(8, 8), (8, 2)], [(8, 2), (2, 2)]]
    x_min = -10
    x_max = 20
    y_min = -10
    y_max = 20
    step_size = 0.5
    max_iter = 1000
    path = rrt(start, goal, obstacles, x_min, x_max, y_min, y_max, step_size, max_iter)
    if path:
        draw_obstacles(obstacles)
        draw_path(path)
        draw_tree(path)
        plt.plot(start.x, start.y, 'bo')
        plt.plot(goal.x, goal.y, 'go')
        plt.axis([x_min, x_max, y_min, y_max])
        plt.show()
    else:
        print('No path found')

if __name__ == '__main__':
    main()