#!/usr/bin/env python
import rospy, tf, math, numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used

#This function consumes linear and angular velocities
#and creates a Twist message.  This message is then published.
def publishTwist(lin_vel, ang_vel):
    global prev_twist
    global pub

    twist_msg = Twist();		#Create Twist Message
    if lin_vel == 0 and ang_vel == 0:
        twist_msg.linear.x = (prev_twist.linear.x)/3
        twist_msg.angular.z = (prev_twist.angular.z)/3
        while twist_msg.linear.x > 0.05 and twist_msg.angular.z > 0.05: 
            twist_msg.linear.x = (prev_twist.linear.x)/3
            twist_msg.angular.z = (prev_twist.angular.z)/3
            prev_twist = twist_msg
            pub.publish(twist_msg)		#Send Message
            rospy.sleep(rospy.Duration(0.2, 0))
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        prev_twist.linear.x = 0
        prev_twist.angular.z = 0
    else:
        twist_msg.linear.x = (2*lin_vel + prev_twist.linear.x)/3
        twist_msg.angular.z = (2*ang_vel + prev_twist.angular.z)/3
        prev_twist = twist_msg
        pub.publish(twist_msg)		#Send Message

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global pose 	
    global start

    x0 = start[0] + pose.pose.position.x	#Set origin
    y0 = start[1] + pose.pose.position.y
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

    print (x0, y0, theta0*(180/math.pi))
    print (dx, dy, theta1*(180/math.pi), distance)
    print (x2, y2, theta2*(180/math.pi))

    print 'Rotate 1'
    rotate(dtheta0)
    rospy.sleep(rospy.Duration(1, 0))
    print 'Straight'
    driveStraight(0.1, distance)
    rospy.sleep(rospy.Duration(1, 0))
    print 'Rotate 2'
    rotate(dtheta1)

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(0.3, .6)
    rotate(math.pi/2)
    driveStraight(0.3, .45)
    rotate(3*math.pi/4)

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    lin_vel = (0.035/2)*(u1 + u2)           #Determines the linear velocity of base based on the wheels
    ang_vel = (0.035/0.23)*(u1 - u2)        #Determines the angular velocity of base on the wheels.

    #While the specified amount of time has not elapsed, send Twist messages.
    now = rospy.Time.now().secs
    cur_lin_vel = 0
    cur_ang_vel = 0
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        publishTwist(lin_vel, ang_vel)
    publishTwist(0, 0)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose 	

    x0 = pose.pose.position.x	#Set origin
    y0 = pose.pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)	#Distance formula
        if (d >= distance):
            done = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose

    #This node was created using Coordinate system transforms and numpy arrays.
    #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    #Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    #Continues creating and matching coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(.5,-.5,.1)
            else:
                spinWheels(-.5,.5,.1)

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    v1 = (speed/0.035)*(radius + .5*.23)
    v2 = (speed/0.035)*(radius - .5*.23)
    time = angle/speed

    spinWheels(v1, v2, time)

#Odometry Callback function.
def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
        (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), 
	rospy.Time.now(),
	"base_footprint","odom")

#Bumper Event Callback function
def readBumper(msg):
    global flag
    if (msg.state == 1):
        flag = 1

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    #pose = Pose()

    #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    #(position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) 
    pass # Delete this 'pass' once implemented


# This is the program's main function
if __name__ == '__main__':
    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    global prev_twist
    global flag
    global pub
    global pose
    global odom_tf
    global odom_list
    global start

    prev_twist = Twist();
    prev_twist.linear.x = 0
    prev_twist.angular.z = 0
    flag = 0
    rospy.init_node('pluxsuwong_lab2')
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
    sub = rospy.Subscriber('/odom', Odometry, readOdom)
    simple_goal_sub = rospy.Subscriber('/move_base_simple/goal/ppp', PoseStamped, navToPose, queue_size=1)
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
    sleeper = rospy.Duration(1)
    rospy.sleep(sleeper)
    #rospy.Timer(rospy.Duration(.01), timerCallback)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    print "Starting Lab 2"

    #make the robot keep doing something...
    #rospy.Timer(rospy.Duration(100), timerCallback)

    # Make the robot do stuff...
    #spinWheels(4, 4, 2)
    #driveStraight(0.3, 0.1)
    #rotate(math.pi)
    '''
    while (flag == 0 and not rospy.is_shutdown()):
        if (flag == 1):
            break
    executeTrajectory()
    '''
    '''
    start = (0, 0)
    while (True and not rospy.is_shutdown()):
        rospy.sleep(rospy.Duration(1, 0))
        pass
    '''
    driveArc(0.5, 0.3, math.pi)
    
    print "Lab 2 complete!"

