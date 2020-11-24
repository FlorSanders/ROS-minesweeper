#!/ur/bin.env python

# Importing required libraries
import rospy as ros
import sys
import time
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Most generic abstract class to handle ROS publishing and subscribing
# Add all available sensors and algorithms to this thing and make subclasses which define the move funcion and achieve greatness
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
        self.odom_sub = None
        self.vel_pub = None

        # Ros parameters
        self.pub_rate = 0.1
        self.queue_size = 2

        # Variables to store sensor information in
        self.position = None
        self.orientation = None

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
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)
        
    def stop_robot(self):
        """
        Callback function for when the robot gets the shutdown command.
        Publishes a stop command to the robot for a secod before shutting down.
        """

        self.t_init = time.time()
    
        while time.time() - self.t_init < 1 and not ros.is_shutdown():
            self.__vel_ros_pub(Twist())
            self.rate.sleep()
        
        sys.exit('The robot has been stopped.')
    
    def go_forward_for(self, tau, v=0.5):
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
            self.__vel_ros_pub(msg)
            self.rate.sleep()

    def go_forward_by(self, d, v):
        """
        Helper function to make the robot drive forward a certain distance at a given speed v
        """
        # Get the initial position
        x_init = self.position.x
        y_init = self.position.y

        # Move by the amount we wanted to move by
        msg = Twist()
        while (self.position.x - x_init)**2 + (self.position.y - y_init)**2 < d**2 and not ros.is_shutdown():
            msg.linear.x = v
            msg.angular.z = 0
            self.__vel_ros_pub(msg)
            self.rate.sleep()

    def get_z_rotation(self, orientation):
        """
        Calculates the current z-angle based on     
        """
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw
    
    def turn_for(self, tau, omega=0.5):
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
            self.__vel_ros_pub(msg)
            self.rate.sleep()

    def turn_by(self, alpha, omega=0.5):
        """
        Helper function to make the robot turn by a certain angle at a given angular speed omega
        """
        # Get the initial angle
        alpha_init = self.get_z_rotation(self.orientation)

        # Move by the angle we wanted to move by
        msg = Twist()
        while abs(self.get_z_rotation(self.orientation) - alpha_init) < abs(alpha) and not ros.is_shutdown():
            msg.angular.z = np.sign(alpha)*omega
            msg.linear.x = 0
            self.__vel_ros_pub(msg)
            self.rate.sleep()
        
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

    def __vel_ros_pub(self, msg):
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
  
    def move(self):
        """
        Function that moves according to the predefined trajectory.
        In this case it is a square loop
        """
        while self.position is None:
            print('Sleeping...')
            self.rate.sleep()
        self.go_forward_by(2, 0.5)
        self.turn_by(np.pi/2, 0.5)
        self.go_forward_by(2, 0.5)
        self.turn_by(np.pi/2, 0.5)
        self.go_forward_by(2, 0.5)
        self.turn_by(np.pi/2, 0.5)
        self.go_forward_by(2, 0.5)
        self.turn_by(np.pi/2, 0.5)
        # Job is done -- Stopping the robot
        self.stop_robot()

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