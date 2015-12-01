#!/usr/bin/env python

import rospy, roslib, time, math, astar, tf, numpy
#from numpy import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

# Generate Waypoints
def genWaypoints(g_path_rev, w_map):
    
    g_pts = []
    w_ori = []
    g_path = list(reversed(g_path_rev))

    # Waypoints based on change in direction
    '''
    prev_pt = g_path[0]
    i_dx = g_path[1].x - g_path[0].x
    i_dy = g_path[1].y - g_path[0].y
    heading = (i_dx, i_dy)
    g_path.pop(0)
    for i, point in enumerate(g_path):
        dx = point.x - prev_pt.x
        dy = point.y - prev_pt.y
        cur_heading = (dx, dy)
        if cur_heading != heading and i != len(g_path) - 1:
            g_pts.append(prev_pt)
            th_heading = math.atan2(heading[1], heading[0])
            q_t = quaternion_from_euler(0, 0, th_heading)
            quat_heading = Quaternion(*q_t)
            w_ori.append(quat_heading)
            heading = cur_heading
        prev_pt = point
    g_pts.append(g_path[-1])
    q_t = quaternion_from_euler(0, 0, 0)
    quat_heading = Quaternion(*q_t)
    w_ori.append(quat_heading)
    '''

    # Waypoints every 4 grids
    prev_pt = g_path[0]
    tmp_ctr = 0
    for i, point in enumerate(g_path):
        if i == len(g_path) - 1:
            g_pts.append(g_path[-1])

            q_t = quaternion_from_euler(0, 0, 0)
            quat_heading = Quaternion(*q_t)

            w_ori.append(quat_heading)
        elif tmp_ctr == 3:
            tmp_ctr = 0
            g_pts.append(point)

            dx = point.x - prev_pt.x
            dy = point.y - prev_pt.y
            heading = (dx, dy)
            th_heading = math.atan2(heading[1], heading[0])
            q_t = quaternion_from_euler(0, 0, th_heading)
            quat_heading = Quaternion(*q_t)

            w_ori.append(quat_heading)
            prev_pt = point
        else:
            tmp_ctr += 1

    # Convert grid points to world coordinates
    w_pts = []
    for point in g_pts:
        w_pts.append(gridToWorld(point, w_map))
    # Create actual Path()
    path = Path()
    for i in range(len(w_pts)):
        path.poses.append(PoseStamped(Header(), Pose(w_pts[i], w_ori[i])))
    return path

'''-----------------------------------------Update Grid Functions---------------------------------------'''

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

'''----------------------------------------Navigation Functions-----------------------------------------'''

# Publish Twist msgs
def publishTwist(lin_vel, ang_vel):
    global prev_twist
    global nav_pub

    twist_msg = Twist();                #Create Twist Message
    if lin_vel == 0 and ang_vel == 0:
        twist_msg.linear.x = (prev_twist.linear.x)/3
        twist_msg.angular.z = (prev_twist.angular.z)/3
        while twist_msg.linear.x > 0.05 and twist_msg.angular.z > 0.05:
            twist_msg.linear.x = (prev_twist.linear.x)/3
            twist_msg.angular.z = (prev_twist.angular.z)/3
            prev_twist = twist_msg
            nav_pub.publish(twist_msg)              #Send Message
            rospy.sleep(rospy.Duration(0.2, 0))
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        nav_pub.publish(twist_msg)
        prev_twist.linear.x = 0
        prev_twist.angular.z = 0
    else:
        twist_msg.linear.x = (2*lin_vel + prev_twist.linear.x)/3
        twist_msg.angular.z = (2*ang_vel + prev_twist.angular.z)/3
        prev_twist = twist_msg
        nav_pub.publish(twist_msg)          #Send Message

# Drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global pose

    x0 = pose.pose.position.x        #Set origin
    y0 = pose.pose.position.y
    q0 = (pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
    x2 = goal.pose.position.x
    y2 = goal.pose.position.y
    q2 = (goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w)
    theta_tup_0 = euler_from_quaternion(q0)
    theta0 = theta_tup_0[2]
    theta_tup_2 = euler_from_quaternion(q2)
    theta2 = theta_tup_2[2]

    dx = x2 - x0
    dy = y2 - y0
    theta1 = math.atan2(dy, dx)

    dtheta0 = theta1 - theta0
    dtheta1 = theta2 - theta1
    distance = math.sqrt(dx**2 + dy**2)
    
    '''
    print "[x0: ", x0, "][y0: ", y0, "][theta0: ", theta0, "]"
    print "[x2: ", x2, "][y2: ", y2, "][theta2: ", theta2, "]"
    print "dtheta0: ", dtheta0
    print "distance: ", distance
    print "dtheta1: ", dtheta1
    '''

    rotate(dtheta0)
    driveStraight(0.1, distance)
    rotate(dtheta1)

def rotate(angle):
    global odom_list
    global pose

    # This node was created using Coordinate system transforms and numpy arrays.
    # The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],     #Create goal rotation
                            [math.sin(angle),  math.cos(angle), 0],
                            [0,                0,          	1]])

    # Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    # Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                          [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                          [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                          [0,             0,             0,             1]])

   # Continues creating and matching coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if (within_tolerance.all()):
            publishTwist(0, 0)
            done = True
        else:
            if (angle > 0):
                publishTwist(0, 0.5)
            else:
                publishTwist(0, -0.5)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose

    x0 = pose.pose.position.x   #Set origin
    y0 = pose.pose.position.y

    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)      #Distance formula
        if (d >= distance):
            publishTwist(0, 0)
            done = True
        else:
            publishTwist(speed, 0)

# Odometry Callback function.
def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
                        (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                        rospy.Time.now(),
                        "base_footprint","odom")

'''-------------------------------------------Main Function---------------------------------------------'''

if __name__ == '__main__':

    # Create Node
    rospy.init_node('lab4')

    # Global Variables
    global new_map_flag
    global world_map
    global goal_pose
    global cost_map

    # Global Variables for Navigation
    global pose
    global odom_tf
    global odom_list
    global prev_twist

    # Initialize Global Variables
    pose = PoseStamped()
    new_map_flag = 0
    world_map = None
    start_pose = None
    goal_pose = None
    cost_map = OccupancyGrid()

    # Initialize Navigation GV
    prev_twist = Twist();
    prev_twist.linear.x = 0
    prev_twist.angular.z = 0

    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    # Subscribers
    world_map_sub = rospy.Subscriber('/map', OccupancyGrid, readWorldMapCallback)
    goal_pose_sub = rospy.Subscriber('/move_base_simple/goal/lab_4', PoseStamped, goalCallback)
    odom_sub = rospy.Subscriber('/odom', Odometry, readOdom)
    # cost_map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, costMapCallback)

    # Publishers
    global path_pub
    global waypoints_pub
    global nav_pub

    path_pub = rospy.Publisher('/lab4/path', GridCells, queue_size=1)
    waypoints_pub = rospy.Publisher('/lab4/waypoints', Path, queue_size=1)
    nav_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

    print "Waiting for map"
    while world_map == None and not rospy.is_shutdown():
        pass
    map_cache = world_map
    world_map = None
    print "Received map"
    
    while not rospy.is_shutdown():
        path = Path()
        flag_cache = new_map_flag
        new_map_flag = 0
        if flag_cache > 0:
            flag_cache = 0
        # Initialization
        
        if world_map != None:
            map_cache = world_map
            world_map = None
            print "Updated map cache"

        start_cache = []
        start_cache.append(pose.pose.position.x)
        start_cache.append(pose.pose.position.y)
        q_tmp = pose.pose.orientation
        quat_tmp = [q_tmp.x, q_tmp.y, q_tmp.z, q_tmp.w]
        r_t, p_t, y_t = euler_from_quaternion(quat_tmp)
        start_cache.append(y_t)

        print "Waiting for new goal position"
        while goal_pose == None and not rospy.is_shutdown():
            if new_map_flag > 0:
                break
        if new_map_flag > 0 or rospy.is_shutdown():
            continue
        goal_cache = goal_pose
        goal_pose = None
        print "Received goal position at: [%f, %f]" % (goal_cache[0], goal_cache[1])

        print "Finished initialization"
        print "Running A* algorithm..."

        # Prepping for A*
        origin_cache = []
        origin_cache.append(map_cache.info.origin.position.x)
        origin_cache.append(map_cache.info.origin.position.y)
        q_tmp = map_cache.info.origin.orientation
        quat_tmp = [q_tmp.x, q_tmp.y, q_tmp.z, q_tmp.w]
        r_t, p_t, y_t = euler_from_quaternion(quat_tmp)
        origin_cache.append(y_t)

        res = map_cache.info.resolution
        map_origin = (int(-origin_cache[0]/res), int(-origin_cache[1]/res))
        start_cc = [int(start_cache[0]/res) + map_origin[0], int(start_cache[1]/res) + map_origin[1], 0]
        goal_cc = [int(goal_cache[0]/res) + map_origin[0], int(goal_cache[1]/res) + map_origin[1], 0]

        # Running A*
        generated_path = astar.a_star(start_cc, goal_cc, map_cache)
        print "Finished running A* algorithm"

        if generated_path != None and not rospy.is_shutdown():
            print "Updated RViz with path"
            rvizPath(generated_path, map_cache)
            path = genWaypoints(generated_path, map_cache)
            waypoints_pub.publish(path)

            print "Published generated path to topic: [/lab4/waypoints]"
        
        print "Waypoints:"
        tmp_wp_ctr = 0
        for waypoint in path.poses:
            print tmp_wp_ctr, ": [", waypoint.pose.position.x, ", ", waypoint.pose.position.y, "]"
            tmp_wp_ctr += 1

        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            tmp_wp_ctr = 0
            curr_cc = []
            for waypoint in path.poses:
                print "Navigating to waypoint: ", tmp_wp_ctr
                tmp_wp_ctr += 1

                # Replanning
                navToPose(waypoint)
                curr_cc = [int(waypoint.pose.position.x/res) + map_origin[0], int(waypoint.pose.position.y/res) + map_origin[1], 0]
                if world_map != None:
                    map_cache = world_map
                    generated_path = astar.a_star(curr_cc, goal_cc, map_cache)

                    print "Updated RViz with path"
                    rvizPath(generated_path, map_cache)
                    path = genWaypoints(generated_path, map_cache)
                    waypoints_pub.publish(path)
                    print "Published generated path to topic: [/lab4/waypoints]"
                    break
            if curr_cc[0] == goal_cc[0] and curr_cc[1] == goal_cc[1]:
                at_goal = True
        
    print "Exiting program"
