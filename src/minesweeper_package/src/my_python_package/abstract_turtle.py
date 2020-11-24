#!/usr/bin/env python

import rospy as ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time


class AbstractTurtle:
    """
    This class is an abstract class to control a turtleBot
    It mainly declares and subscribes to ROS topics in an elegant way

    For more information:
    https://www.geeksforgeeks.org/abstract-classes-in-python/
    """

    def __init__(self):
        self.node_name = "turtlebot3"

        # Declare ROS subscribers and publishers
        self.odom_sub_name = "/odom"
        self.scan_sub_name = "/scan"
        self.vel_pub_name = "/cmd_vel"

        # Initialize placeholders for subscribers and publishers
        # It is customary in python to define all public variables 
        # (i.e. the ones starting with "self") in __init__(), even
        # though we don't want to give them a value yet. Therefore
        # we initialize them with value None
        self.odom_sub, self.scan_sub, self.vel_pub = (None,) * 3

        # Initialize placeholders for subscriber messages
        self.odom_msg, self.scan_msg = (None,) * 2

        # ROS parameters
        self.pub_rate = 0.1
        self.queue_size = 2

    def start_ros(self):

        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Create a callback to stop the robot when we interrupt the 
        # program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create the subscribers and publishers
        self.odom_sub = ros.Subscriber(
            self.odom_sub_name, Odometry,
            callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.scan_sub = ros.Subscriber(
            self.scan_sub_name, LaserScan,
            callback=self.__scan_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(
            self.vel_pub_name, Twist, queue_size=self.queue_size)

        # Wait for our python program to receive its first messages
        while any([msg is None for msg in 
                   (self.odom_msg, self.scan_msg)]) and \
                not ros.is_shutdown():
            time.sleep(self.pub_rate)

    def stop_robot(self):
        # Get the initial time
        t_init = time.time()

        # We publish for a second to be sure the robot receives the
        # message
        while time.time() - t_init < 1 and not ros.is_shutdown():
            self.vel_ros_pub(Twist())  # empty Twist => do nothing
            time.sleep(self.pub_rate)

    def run(self):
        """To be overridden in the inheriting class"""
        while not ros.is_shutdown():
            time.sleep(1)

    def __odom_ros_sub(self, msg):
        self.odom_msg = msg

    def __scan_ros_sub(self, msg):
        self.scan_msg = msg

    def vel_ros_pub(self, msg):
        self.vel_pub.publish(msg)