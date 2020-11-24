#!/ur/bin.env python

# Importing required libraries
import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Most generic abstract class to handle ROS publishing and subscribing
class RobotController(object):
    """
    Abstract class to control a trajectory on the turtlebot.
    Abstracts away the declaration of ros messages and subscription to ros topics for the rest of the program
    """

    def __init__(self):
        """"
        Initialization with definition for the subscribers and publishers as well as some general parameters and variables.
        """
        # ROS subscribers and publishers
        self.node_name = "square_trajectory"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel"
        self.vel_pub = None
        self.odom_sub = None

        # Ros parameters
        self.pub_rate = 5
        self.queue_size = 2

        # Variables to store sensor information in
        self.odom_pose = None

    def start_ros(self):
        """
        Handle creation of ros nodes, making subscribers and publishers and define callback on shutdown.
        """
        # Present the script as a node in ros (anonymous = True allows the simultaneous running of multiple of these scripts)
        ros.init_node(self.node_name, log_level=ros.INFO, anonymous=True)

        # Setting ros rate
        self.rate = ros.Rate(self.pub_rate)

        # Callback function on shutdown
        ros.on_shutdown(self.stop_robot)

        # Subscribers and publishers
        self.odom_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)
        
    def stop_robot(self):
        """
        Callback function for when the robot gets the shutdown command.
        Publishes a stop command to the robot for a secod before shutting down.
        """

        self.t_init = time.time()
    
        while time.time() - self.t_init < 1 and not ros.is_shutdown():
            self.vel_ros_pub(Twist())
            self.rate.sleep()
        
        sys.exit('The robot has been stopped.')

    def move(self):
        """
        To be overwritten in the inhereting class!
        """

        while not ros.is_shutdown():
            self.rate.sleep()
    
    def __odom_ros_sub(self, msg):
        """
        Handles subscription for the odometry topic.
        """
        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):
        """
        Handles publishing of 'msg' for the velocity topic.
        """
        self.vel_pub.publish(msg)

# Subclass of the robot controller that implements a square open loop driving move function
class OpenLoopDriver(RobotController):
    """
    Class implements an open loop square trajectory based on velocity control.
    It uses the Robotcontroller as parent class to handle the ros intricacies.
    """

    def __init___(self):
        """
        Initialize the open loop driver controller.
        """
        # Simply use the initialization of the superclass
        super(OpenLoopDriver, self).__init__()
    
    def go_forward(self, tau, v):
        """
        Helper function to make the robot drive forward forward at speed v for a given duration tau
        """
        # Set the initial time
        self.t_init = time.time()
        
        # While we don't go over duration, move forward
        msg = Twist()
        while time.time() - self.t_init < tau and not ros.is_shutdown():
            msg.linear.x = v
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            self.rate.sleep()

    def turn(self, tau, omega):
        """
        Helper function to make the robot turn at angular speed omega for a given duration tau
        """
        # Set the initial time
        self.t_init = time.time()

        # While we don't go over duration, turn
        msg = Twist()
        while time.time() - self.t_init < tau and not ros.is_shutdown():
            msg.linear.x = 0
            msg.angular.z = omega
            self.vel_ros_pub(msg)
            self.rate.sleep()
    
    def move(self):
        """
        Function that moves according to the predefined trajectory.
        In this case it is a square loop
        """
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)

# Run the open loop driver if this script is called
if __name__  == "__main__":
    try:
        # Start the contoller
        controller = OpenLoopDriver()
        controller.start_ros()
        controller.move()
    except ros.ROSInterruptException:
        # We do this to avoid accidentally continuing to run code after the module is shut down
        pass