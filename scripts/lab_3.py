#!/usr/bin/env python

import rospy
import roslib
import time
import math
import astar
from numpy import *
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

'''---------------------------------------Callback Functions--------------------------------------------'''

def readWorldMapCallback(data):
    global world_map
    global new_map_flag
    global start_pose
    global goal_pose

    world_map = data
    start_pose = None
    goal_pose = None
    new_map_flag = 1

def startCallback(data):
    global start_pose
    tmp_buf = []
    pos_buf = data.pose.pose.position
    tmp_buf.append(pos_buf.x)
    tmp_buf.append(pos_buf.y)
    q = data.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    r, p, y = euler_from_quaternion(quat)
    tmp_buf.append(y)
    start_pose = tmp_buf

def goalCallback(data):
    global goal_pose
    tmp_buf = []
    pos_buf = data.pose.position
    tmp_buf.append(pos_buf.x)
    tmp_buf.append(pos_buf.y)
    q = data.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    r, p, y = euler_from_quaternion(quat)
    tmp_buf.append(y)
    goal_pose = tmp_buf

def costMapCallback(data):
    global cost_map
    cost_map = data

'''-----------------------------------------Helper Functions--------------------------------------------'''

# (real world) -> (grid)
def worldToGrid(w_pt, w_map):
    res = w_map.info.resolution
    g_pt = Point()
    g_pt.x = int((w_pt.x - w_map.info.origin.position.x)/res) 
    g_pt.y = int((w_pt.y - w_map.info.origin.position.y)/res)
    return g_pt

# (grid) -> (real world)
def gridToWorld(g_pt, w_map):
    res = w_map.info.resolution
    w_pt = Point()
    w_pt.x = ((g_pt.x*res) + w_map.info.origin.position.x) + (0.5*res)
    w_pt.y = ((g_pt.y*res) + w_map.info.origin.position.y) + (0.5*res)
    return w_pt

'''-----------------------------------------Update Grid Functions---------------------------------------'''

def rvizStart(pos_buf, w_map):
    global start_pub

    start_point = Point(pos_buf[0], pos_buf[1], 0)
    start_GC = GridCells()
    start_GC.cell_width = w_map.info.resolution
    start_GC.cell_height = w_map.info.resolution
    start_GC.cells = [gridToWorld(worldToGrid(start_point, w_map), w_map)]
    start_GC.header.frame_id = 'map'
    start_pub.publish(start_GC)

def rvizGoal(pos_buf, w_map):
    global goal_pub

    goal_point = Point(pos_buf[0], pos_buf[1], 0)
    goal_GC = GridCells()
    goal_GC.cell_width = w_map.info.resolution
    goal_GC.cell_height = w_map.info.resolution
    goal_GC.cells = [gridToWorld(worldToGrid(goal_point, w_map), w_map)]
    goal_GC.header.frame_id = 'map'
    goal_pub.publish(goal_GC)

def rvizPath(cell_list, w_map):
    global path_pub

    path_GC = GridCells()
    path_GC.cell_width = w_map.info.resolution
    path_GC.cell_height = w_map.info.resolution
    path_GC.cells = []
    for cell in cell_list:
        path_GC.cells.append(gridToWorld(cell, w_map))
    path_GC.header.frame_id = 'map'
    path_pub.publish(path_GC)

'''-------------------------------------------Main Function---------------------------------------------'''

if __name__ == '__main__':

    # Global Variables
    global new_map_flag
    global world_map
    global start_pose
    global goal_pose
    global cost_map

    # Initialize Global Variables
    new_map_flag = 0
    world_map = None
    start_pose = None
    goal_pose = None
    cost_map = OccupancyGrid()

    # Create Node
    rospy.init_node('lab3')

    # Subscribers
    world_map_sub = rospy.Subscriber('/map', OccupancyGrid, readWorldMapCallback)
    start_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, startCallback)
    goal_pose_sub = rospy.Subscriber('/move_base_simple/goal/lab_3', PoseStamped, goalCallback)
    cost_map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, costMapCallback)

    # Publishers
    global start_pub
    global goal_pub
    global path_pub
    global waypoints_pub

    start_pub = rospy.Publisher('/lab3/start', GridCells, queue_size=1)
    goal_pub = rospy.Publisher('/lab3/goal', GridCells, queue_size=1)
    path_pub = rospy.Publisher('/lab3/path', GridCells, queue_size=1)
    waypoints_pub = rospy.Publisher('/lab3/waypoints', Path, queue_size=1)

    print "Waiting for map"
    while world_map == None and not rospy.is_shutdown():
        pass
    map_cache = world_map
    world_map = None
    print "Received map"
    
    while not rospy.is_shutdown():
        flag_cache = new_map_flag
        new_map_flag = 0
        if flag_cache > 0:
            flag_cache = 0
        # Initialization
        
        if world_map != None:
            map_cache = world_map
            world_map = None
            print "Updated map cache"

        print "Waiting for new start position"
        while start_pose == None and not rospy.is_shutdown():
            if new_map_flag > 0:
                break
        if new_map_flag > 0 or rospy.is_shutdown():
            continue
        start_cache = start_pose
        start_pose = None
        rvizStart(start_cache, map_cache)
        print "Received start position at: [%f, %f]" % (start_cache[0], start_cache[1])

        print "Waiting for new goal position"
        while goal_pose == None and not rospy.is_shutdown():
            if new_map_flag > 0:
                break
        if new_map_flag > 0 or rospy.is_shutdown():
            continue
        goal_cache = goal_pose
        goal_pose = None
        rvizGoal(goal_cache, map_cache)
        print "Received goal position at: [%f, %f]" % (goal_cache[0], goal_cache[1])

        print "Finished initialization"
        print "Running A* algorithm..."
        res = map_cache.info.resolution
        start_cc = [int(start_cache[0]/res), int(start_cache[1]/res), start_cache[2]]
        goal_cc = [int(goal_cache[0]/res), int(goal_cache[1]/res), goal_cache[2]]

        generated_path = astar.a_star(start_cc, goal_cc, map_cache)
        print "Finished running A* algorithm"

        if generated_path != None and not rospy.is_shutdown():
            print "Updated RViz with path"
            rvizPath(generated_path, map_cache)
            
            path = Path()
            for n in generated_path:
                path.poses.append(PoseStamped(Header(), Pose(n, Quaternion())))
            waypoints_pub.publish(path)

            print "Published generated path to topic: [/lab3/waypoints]"
    
    print "Exiting program"
