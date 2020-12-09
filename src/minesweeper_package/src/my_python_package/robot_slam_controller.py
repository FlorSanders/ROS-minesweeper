#!/usr/bin.env python

# Importing required libraries
import rospy as ros
import tf
import sys
import numpy as np
import actionlib

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
        self.map_sub_name = "/map"
        # Rose subscribers and publishers
        self.odom_sub = None
        self.map_sub = None
        self.vel_pub = None

        # Listener for the transformations
        self.transformer = None

        # Ros parameters
        self.pub_rate = 0.05
        self.queue_size = 2

        # Variables to store sensor information in
        self.position = None
        self.orientation = None
        self.map = None
        self.map_dim = None
        self.map_res = None
        self.map_origin = None
        self.map_time = None

        # Debugging option
        self.debug = debug

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
        self.map_sub = ros.Subscriber(self.map_sub_name, OccupancyGrid, callback=self.__map_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

        # Listener for the transformations
        self.transformer = tf.TransformListener()

        # Now wait until the map is made
        while any([msg is None for msg in [self.position, self.orientation, self.lds_ranges, self.map, self.map_dim, self.map_res, self.map_origin, self.map_time]]) and not ros.is_shutdown():
            self.printd('Sleeping...')
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
    
    def __map_ros_sub(self, msg):
        """
        Handles subscription for the map topic.
        """
        self.map_dim = [msg.info.width, msg.info.height]
        self.map_res = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.map_time = np.array([msg.header.stamp.secs, msg.header.stamp.nsecs])
        self.map = np.array(msg.data).reshape(self.map_dim)

    def __vel_ros_pub(self, msg):
        """
        Handles publishing of 'msg' for the velocity topic.
        """
        self.vel_pub.publish(msg)