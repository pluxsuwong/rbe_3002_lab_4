#!/usr/bin/env python

import rospy
import roslib
import math
import lab_4
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point

# start_in = [x, y, yaw]
# goal_in = [x, y, yaw]
# w_map = OccupancyGrid()
def a_star(start_in, goal_in, w_map):

    global expanded_pub
    global frontier_pub
    global unexplored_pub
    global obstacles_pub

    expanded_pub = rospy.Publisher('/lab4/expanded', GridCells, queue_size=0)
    frontier_pub = rospy.Publisher('/lab4/frontier', GridCells, queue_size=0)
    unexplored_pub = rospy.Publisher('/lab4/unexplored', GridCells, queue_size=0)
    obstacles_pub = rospy.Publisher('/lab4/obstacles', GridCells, queue_size=1)

    start = Point(start_in[0], start_in[1], start_in[2])
    goal = Point(goal_in[0], goal_in[1], goal_in[2])
    map_len = w_map.info.width
    start_i = p_to_i(start, map_len)
    # print "start: ", start, "start_i: ", start_i
    goal_i = p_to_i(goal, map_len)
    # print "goal: ", goal, "goal_i: ", goal_i

    closed_set = set([])    	  # The set of nodes already evaluated.
    open_set = set([])    # The set of tentative nodes to be evaluated, initially containing the start node
    open_set.add(start_i)

    came_from = {}

    e_o_i = rvizObstacles(w_map)
    print "Expanded obstacles on map"

    w_map_2 = updateMap(e_o_i, w_map)

    g_score = []
    for x in range(len(w_map_2.data)):
        g_score.append(99999.9)
    g_score[start_i] = 0    # Cost from start along best known path.

    # Estimated total cost from start to goal through y.
    f_score = []
    for x in range(len(w_map_2.data)):
        f_score.append(99999.9)
    f_score[start_i] = g_score[start_i] + heuristic_cost_estimate(start_i, goal_i, map_len)
     
    while len(open_set) > 0 and not rospy.is_shutdown():
        current_i = int(min_f_score_node(open_set, f_score, map_len))  # the node in open_set having the lowest f_score[] value
        if current_i == goal_i:

            rvizExpanded([0], w_map_2)
            rvizFrontier([0], w_map_2)
            return reconstruct_path(came_from, goal_i, map_len)
         
        open_set.discard(current_i)
        closed_set.add(current_i)
        current_neighbors = neighbors(current_i, w_map_2)
        
        for neighbor_i in current_neighbors:
            if neighbor_i in closed_set:
                continue		# Ignore the neighbor which is already evaluated.
            tentative_g_score = g_score[current_i] + heuristic_cost_estimate(current_i, neighbor_i, map_len) # length of this path.
            if neighbor_i not in open_set:	# Discover a new node
                open_set.add(neighbor_i)
            elif tentative_g_score >= g_score[neighbor_i]:
                continue		# This is not a better path.

            # This path is the best until now. Record it!
            came_from[neighbor_i] = current_i
            g_score[neighbor_i] = tentative_g_score
            f_score[neighbor_i] = g_score[neighbor_i] + heuristic_cost_estimate(neighbor_i, goal_i, map_len)

        rvizFrontier(open_set, w_map_2)
        rvizExpanded(closed_set, w_map_2)

    print 'A* Failed...'
    return None

def reconstruct_path(came_from, goal_i, w):
    total_path = []
    
    goal = i_to_p(goal_i, w)
    current_i = goal_i
    total_path.append(i_to_p(current_i, w))
    while current_i in came_from.keys() and not rospy.is_shutdown():
        current_i = came_from[current_i]
        total_path.append(i_to_p(current_i, w))
    return total_path

def p_to_i(point, width):
    return int(point.x + width*point.y)

def i_to_p(index, width):
    p_x = index % width
    p_y = index / width
    point = Point(p_x, p_y, 0)
    return point

def heuristic_cost_estimate(start_i, goal_i, w):
    start = i_to_p(start_i, w)
    goal = i_to_p(goal_i, w)
    distance = math.sqrt((start.x - goal.x)**2 + (start.y - goal.y)**2)
    return distance

def min_f_score_node(open_set, f_score, w):
    min_n = -1
    min_score = 999
    for n in open_set:
        score_buf = f_score[n]
        if score_buf < min_score:
            min_n = n
            min_score = score_buf
    return min_n

def neighbors(node_i, w_map):
    n_list = []
    node = i_to_p(node_i, w_map.info.width)
    w = w_map.info.width
    h = w_map.info.height

    if node.x > 0 and w_map.data[int((node.x - 1) + w*node.y)] < 50:
        n_list.append(int(node.x - 1) + w*int(node.y))
    if node.x < w and w_map.data[int((node.x + 1) + w*node.y)] < 50:
        n_list.append(int(node.x + 1) + w*int(node.y))
    if node.y > 0 and w_map.data[int(node.x + w*(node.y - 1))] < 50:
        n_list.append(int(node.x) + w*int(node.y - 1))
    if node.y < h and w_map.data[int(node.x + w*(node.y + 1))] < 50:
        n_list.append(int(node.x) + w*int(node.y + 1))

    if node.x > 0 and node.y > 0 and w_map.data[int((node.x - 1) + w*(node.y - 1))] < 50:
        n_list.append(int(node.x - 1) + w*int(node.y - 1))
    if node.x < w and node.y < h and w_map.data[int((node.x + 1) + w*(node.y + 1))] < 50:
        n_list.append(int(node.x + 1) + w*int(node.y + 1))
    if node.y > 0 and node.x < w and w_map.data[int((node.x + 1) + w*(node.y - 1))] < 50:
        n_list.append(int(node.x + 1) + w*int(node.y - 1))
    if node.y < h and node.x > 0 and w_map.data[int((node.x - 1) + w*(node.y + 1))] < 50:
        n_list.append(int(node.x - 1) + w*int(node.y + 1))
    
    return n_list

def rvizExpanded(cell_list, w_map):
    global expanded_pub

    expanded_GC = GridCells()
    expanded_GC.cell_width = w_map.info.resolution
    expanded_GC.cell_height = w_map.info.resolution
    expanded_GC.cells = []
    for cell in cell_list:
        expanded_GC.cells.append(lab_4.gridToWorld(i_to_p(cell, w_map.info.width), w_map))
    expanded_GC.header.frame_id = 'map'
    expanded_pub.publish(expanded_GC)

def rvizFrontier(cell_list, w_map):
    global frontier_pub

    frontier_GC = GridCells()
    frontier_GC.cell_width = w_map.info.resolution
    frontier_GC.cell_height = w_map.info.resolution
    frontier_GC.cells = []
    for cell in cell_list:
        frontier_GC.cells.append(lab_4.gridToWorld(i_to_p(cell, w_map.info.width), w_map))
    frontier_GC.header.frame_id = 'map'
    frontier_pub.publish(frontier_GC)

def rvizUnexplored(cell_list, w_map):
    global unexplored_pub

    unexplored_GC = GridCells()
    unexplored_GC.cell_width = w_map.info.resolution
    unexplored_GC.cell_height = w_map.info.resolution
    unexplored_GC.cells = []
    for cell in cell_list:
        unexplored_GC.cells.append(lab_4.gridToWorld(i_to_p(cell, w_map.info.width), w_map))
    unexplored_GC.header.frame_id = 'map'
    unexplored_pub.publish(unexplored_GC)

def rvizObstacles(w_map):
    global obstacles_pub

    obstacles_GC = GridCells()
    obstacles_GC.cell_width = w_map.info.resolution
    obstacles_GC.cell_height = w_map.info.resolution
    obstacles_GC.cells = []
    obstacle_pts = []
    expanded_pts = set([])
    e_o_i = set([])

    for index, node in enumerate(w_map.data):
        if node > 50:
            obstacle_pts.append(index)

    for index in obstacle_pts:
        for i in range(-4, 5):
            for j in range(-4, 5):
                point = i_to_p(index, w_map.info.width)
                point.x += i
                point.y += j
                e_o_i.add(p_to_i(point, w_map.info.width))
                expanded_pts.add(lab_4.gridToWorld(point, w_map))
    
    obstacles_GC.cells = list(expanded_pts)

    obstacles_GC.header.frame_id = 'map'
    obstacles_pub.publish(obstacles_GC)
    return list(e_o_i)

def updateMap(e_o_indexes, w_map):
    newGrid = OccupancyGrid()
    newGrid.info = w_map.info
    tmp_data = list(w_map.data)
    
    for index in e_o_indexes:
        if index >= 0 and index < len(w_map.data):
            tmp_data[index] = 100

    newGrid.data = tuple(tmp_data)

    return newGrid
