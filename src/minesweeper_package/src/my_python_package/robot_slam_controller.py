#!/usr/bin.env python

# Importing required libraries
import rospy as ros
import tf
import sys
import numpy as np
import actionlib
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Most generic abstract class to handle ROS publishing and subscribing, making use of SLAM map data and the navigation stack in order to move around
# Add all available sensors and algorithms to this thing and make subclasses which define the move funcion and achieve greatness
class RobotSlamController(object):
    """
    Abstract class to control a trajectory on the turtlebot, based on mapping data.
    Abstracts away the declaration of ros messages and subscription to ros topics for the rest of the program
    """

    def __init__(self, debug=True):
        """"
        Initialization with definition for the subscribers and publishers as well as some general parameters and variables.
        """
        # Name of our node
        self.node_name = "robot_slam_controller"
        # ROS subscription and publication topics
        self.odom_sub_name = "/odom"
        self.lds_sub_name = "/scan"
        self.map_sub_name = "/map"
        self.vel_pub_name = "/cmd_vel"
        # Rose subscribers and publishers
        self.odom_sub = None
        self.lds_sub = None
        self.map_sub = None
        self.vel_pub = None
        # Move base action client
        self.action_client = None

        # Listener for the transformations
        self.transformer = None

        # Ros parameters
        self.pub_rate = 0.05
        self.queue_size = 2
        self.robot_radius = 0.1

        # Variables to store sensor information in
        self.position = None
        self.orientation = None
        self.lds_ranges = None
        # Variables to do with the mapping process
        self.map = None
        self.map_update = False
        self.map_dim = None
        self.map_res = None
        self.map_origin = None
        self.map_time = None
        self.map_visited = None
        self.map_visited_origin = None
        self.position_on_map = None
        self.previous_on_map = None
        self.orientation_on_map = None
        self.goal_on_map = None
        self.goal_orientation_on_map = None

        # Debugging option
        self.debug = debug

        if self.debug:
            # Start a new figure to visualize our data
            plt.figure(figsize = (30,30))

    def printd(self, msg):
        """
        Function that prints only if debug is active
        """
        if self.debug:
            ros.loginfo(str(msg))

    def start_ros(self):
        """
        Handle creation of ros nodes, making subscribers and publishers and define callback on shutdown.
        """
        # Present the script as a node in ros (anonymous = True allows the simultaneous running of multiple of these scripts)
        ros.init_node(self.node_name, log_level=ros.INFO, anonymous=True)

        # Setting ros rate
        self.rate = ros.Rate(1/self.pub_rate)

        # Callback function on shutdown
        ros.on_shutdown(self.stop_robot)

        # Subscribers and publishers
        self.odom_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.lds_sub = ros.Subscriber(self.lds_sub_name, LaserScan, callback=self.__lds_ros_sub, queue_size=self.queue_size)
        self.map_sub = ros.Subscriber(self.map_sub_name, OccupancyGrid, callback=self.__map_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

        # Listener for the transformations
        self.transformer = tf.TransformListener()

        # Initialize the action client for move_base goals and wait for server startup
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.printd('Waiting for action server to become available...')
        self.action_client.wait_for_server()

        # Now wait until all topics have received at least one message
        while any([msg is None for msg in [self.position, self.orientation, self.lds_ranges, self.map, self.map_dim, self.map_res, self.map_origin, self.map_time]]) and not ros.is_shutdown():
            self.printd('Waiting for messages to arrive...')
            self.rate.sleep()
        
    def stop_robot(self):
        """
        Callback function for when the robot gets the shutdown command.
        Publishes a stop command to the robot for a secod before shutting down.
        """

        self.t_init = ros.get_time()
    
        while ros.get_time() - self.t_init < 1 and not ros.is_shutdown():
            self.__vel_ros_pub(Twist())
            self.rate.sleep()
        
        sys.exit('The robot has been stopped.')

    def set_goal_angle(self):
        """
        Returns the orientation the robot should have to be turned towards the goal
        """
        vector = self.goal_on_map - self.position_on_map
        alpha = np.arctan(vector[1]/vector[0])
        if vector[0] < 0:
            alpha = np.pi+alpha
        self.goal_orientation_on_map = alpha

    def send_goal(self):
        """
        Send the goal saved in goal_on_map as a goal to the move_base client.
        As the orientation is not all that important to us, we'll use the current orientation of the robot (which hopefully requires little turning)
        """
        # Creating new map
        goal = MoveBaseGoal()
        # Picking reference frame
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = ros.Time.now()
        # Defining how many meters we want to move in which direction with respect to the map reference frame (just move 0.5m along x direction for now)
        goal.target_pose.pose.position.x = self.goal_on_map[0]*self.map_res + self.map_origin[0]
        goal.target_pose.pose.position.y = self.goal_on_map[1]*self.map_res + self.map_origin[1]
        # Using the ideal "straight line" orientation computed earlier as the goal orientation
        goal_orientation = tf.transformations.quaternion_about_axis(self.goal_orientation_on_map, (0,0,1))
        goal.target_pose.pose.orientation.x = goal_orientation[0]
        goal.target_pose.pose.orientation.y = goal_orientation[1]
        goal.target_pose.pose.orientation.z = goal_orientation[2]
        goal.target_pose.pose.orientation.w = goal_orientation[3]
        # Sending the goal using our client
        self.action_client.send_goal(goal)
        wait = self.action_client.wait_for_result()
        if not wait:
            self.printd('Action server not available')
            self.stop_robot()
        else:
            return self.action_client.get_result()
        
    def move(self):
        """
        Function that moves according to the predefined strategy.
        To be overwritten in the inhereting class!
        """
        while not ros.is_shutdown():
            self.rate.sleep()
    
    def __odom_ros_sub(self, msg):
        """
        Handles subscription for the odometry topic.
        """
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
    
    def __lds_ros_sub(self, msg):
        """
        Handles subscription for the Light Distance Sensor topic.
        """
        self.lds_ranges = msg.ranges
    
    def __update_map_visited(self):
        """
        Builds or updates the visited map based on the map data we just obtained
        """
        # Checking if we already have a map
        if np.any(self.map_visited == None):
            # Uninitialised map ==> fresh start
            self.map_visited = np.zeros(self.map_dim, dtype=bool)
        
        # Updating the visited points on the map
        coordinates = np.indices(self.map_dim).transpose((2,1,0))
        if np.any(self.previous_on_map == None):
            self.map_visited[np.sum((coordinates - self.position_on_map)**2, -1) < (self.robot_radius/self.map_res)**2] = True
        else:
            n_points = 10
            direction = (self.position_on_map - self.previous_on_map)/(n_points-1)
            for i in range(n_points):
                point = self.previous_on_map + i*direction
                self.map_visited[np.sum((coordinates - point)**2, -1) < (self.robot_radius/self.map_res)**2] = True
        
        # If the map itself has changed --> Apply corrections as well
        if self.map_visited.shape != self.map.shape or np.any(self.map_visited_origin != self.map_origin):
            self.printd("Map has changed!!")
            # Correct the size of the map
            ## TODO --> Necessary?
            # Corrected the origin of the map
            self.map_visited_origin = self.map_origin
    
    def __get_pose_on_map(self):
        """
        Performs all the required transformations from the mapping data to derive our pose on the map
        """
        # Obtain the (transformed) position and rotation from the topics
        try:
            translation, rotation = self.transformer.lookupTransform('/map', 'base_footprint', ros.Time(self.map_time[0], self.map_time[1]))
            self.previous_on_map = self.position_on_map
            self.position_on_map = (np.array(translation[0:2]) - self.map_origin) / self.map_res
            _,_,self.orientation_on_map = tf.transformations.euler_from_quaternion(rotation)
            return True
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            # Something could go wrong in transformation
            self.printd(err)
            return False

    def __map_ros_sub(self, msg):
        """
        Handles subscription for the map topic.
        """
        self.printd('Received map')
        # Getting the information that's immediately available in the message
        self.map_dim = [msg.info.width, msg.info.height]
        self.map_res = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.map_time = np.array([msg.header.stamp.secs, msg.header.stamp.nsecs])
        self.map = np.array(msg.data).reshape(self.map_dim)
        # Get the pose on the new map we reveived
        success = self.__get_pose_on_map()
        # Update the places we visited based on all information we have, if previous operation was succeful
        if success:
            self.printd('Success')
            self.__update_map_visited()
            # Plot the map, if debugging is enabled
            if self.debug:
                self.printd('Map can be plotted now')
                self.map_update = True

    def __vel_ros_pub(self, msg):
        """
        Handles publishing of 'msg' for the velocity topic.
        """
        self.vel_pub.publish(msg)