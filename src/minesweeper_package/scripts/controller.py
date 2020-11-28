#!/usr/bin.env python

# Importing required libraries
import rospy as ros
import time
import sys
import numpy as np
import argparse

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Most generic abstract class to handle ROS publishing and subscribing
# Add all available sensors and algorithms to this thing and make subclasses which define the move funcion and achieve greatness
class RobotController(object):
    """
    Abstract class to control a trajectory on the turtlebot.
    Abstracts away the declaration of ros messages and subscription to ros topics for the rest of the program
    """

    def __init__(self, tau=0.25, v=1.0, omega=1.0, alpha=np.pi/2, d=1.0, debug = False):
        """"
        Initialization with definition for the subscribers and publishers as well as some general parameters and variables.
        """
        # Name of our node
        self.node_name = "robot_controller"
        # ROS subscription and publication topics
        self.odom_sub_name = "/odom"
        self.lds_sub_name = "/scan"
        self.vel_pub_name = "/cmd_vel"
        # Rose subscribers and publishers
        self.odom_sub = None
        self.lds_sub = None
        self.vel_pub = None

        # Ros parameters
        self.pub_rate = 0.1
        self.queue_size = 2

        # Variables to store sensor information in
        self.position = None
        self.orientation = None
        self.lds_ranges = None

        # Velocity, distance and timing parameters
        self.tau = tau
        self.v = v
        self.omega = omega
        self.alpha = alpha
        self.d = d

        # Debugging option
        self.debug = debug

    def printd(self, msg):
        """
        Function that prints only if debug is active
        """
        if self.debug:
            print(msg)

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
        self.t_init = ros.get_time()
        
        # While we don't go over duration, move forward
        msg = Twist()
        while ros.get_time() - self.t_init < tau and not ros.is_shutdown():
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
        self.t_init = ros.get_time()

        # While we don't go over duration, turn
        msg = Twist()
        while ros.get_time() - self.t_init < tau and not ros.is_shutdown():
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
    
    def __lds_ros_sub(self, msg):
        """
        Handles subscription for the Light Distance Sensor topic.
        """
        self.lds_ranges = msg.ranges

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

    def __init___(self, **params):
        """
        Initialize the open loop driver controller.
        """
        # Simply use the initialization of the superclass
        super(OpenLoopDriver, self).__init__(**params)
  
    def move(self):
        """
        Function that moves according to the predefined trajectory.
        In this case it is a square loop
        """
        while self.position is None:
            self.printd('Sleeping...')
            self.rate.sleep()
        for i in range(4):
            self.go_forward_by(self.d, self.v)
            self.turn_by(self.alpha, self.omega)
        self.turn_by(np.pi/2, 0.5)
        # Job is done -- Stopping the robot
        self.stop_robot()

# First "minesweeper strategy": random movement with obstacle avoidance
class RandomRoomba(RobotController):
    """
    Class implements a random trajectory with obstacle avoidance, based on the Laser Distance Sensor (LDS).
    The principle is inspired on the commercially available Roomba vacuum cleaners
    """

    def __init__(self, **params):
        """
        Initialize the roomba.
        """
        # Use the superclass initialization function
        super(RandomRoomba, self).__init__(**params)
    
    def scan_for_obstacles(self):
        """
        Uses the LDS to scan for obstacles that are in our path.
        Returns whether we are close to an object (< 0.5m) as well as the full array of ranges.
        """
        # Fetching our ranges
        ranges = np.array(list(self.lds_ranges))
        
        # If nothing is detected, the lds returns zero. Changing this by a long distance (4m)
        ranges[ranges == 0.] = 4.

        # Select ranges in the direction we are driving
        relevant_ranges = np.concatenate((ranges[-12:], ranges[:12])) # Not too sure about these indices
        blocked = np.any(relevant_ranges < 0.5)

        self.printd((np.argmin(ranges), np.min(ranges), len(ranges), blocked))

        # Return whether we are close to an object (within 0.5m) together with the whole range
        return blocked, ranges

    def move(self):
        """
        1. Wait until readings from the sensor are available.
        2. Moves straight until the sensor detects possible collision.
        3. Turns towards an unblocked direction and continues.
        """
        # Wait until sensor readings are available
        while (self.position is None or self.lds_ranges is None) and not ros.is_shutdown():
            self.printd('Sleeping...')
            self.rate.sleep()

        # If these are available, move straight until blocked or unblock
        while not ros.is_shutdown():
            blocked, ranges = self.scan_for_obstacles()
            
            if blocked:
                unblocked_directions = np.linspace(-np.pi, np.pi, len(ranges))[ranges >= 0.5]
                random_choice = np.random.choice(unblocked_directions, size=1)
                self.turn_by(random_choice, omega=self.omega)
            else:
                self.go_forward_for(tau=self.tau, v=self.v)


strategies = {
    'squareloop': OpenLoopDriver,
    'randomroomba': RandomRoomba,
}

# Run the open loop driver if this script is called
if __name__  == "__main__":
    # Helper library that parses command line option
    parser = argparse.ArgumentParser(description = 'Minesweeper controller for our robot')
    parser.add_argument('strategy', 
        help=(f"Which minesweeping strategy you want to use, Options: [{'/'.join(strategies.keys())}]"),
        nargs=1,
        type=str
    )
    parser.add_argument('--speed',
        help='Linear speed of the robot when moving',
        nargs='?',
        default=0.5,
        type=float
    )
    parser.add_argument('--angularspeed',
        help='Angular speed of the robot when turning',
        nargs='?',
        default=1.0,
        type=float
    )
    parser.add_argument('--duration',
        help='Default step duration the robot moves for',
        nargs='?',
        default=0.25,
        type=float
    )
    parser.add_argument('--angle',
        help='Default angle the robot rotates by',
        nargs='?',
        default=np.pi/2,
        type=float
    )
    parser.add_argument('--distance',
        help='Default distance the robot moves by',
        nargs='?',
        default=1.0,
        type=float
    )
    parser.add_argument('--debug',
        help='Writes debug messages using print() when added',
        action='store_true'
    )

    # Parsing sys.argv
    args = parser.parse_args()
    print(args)
    
    # Launch the controller
    try:
        if args.strategy[0] in list(strategies.keys()):
            controller = strategies[args.strategy[0]](tau = args.duration, v=args.speed, omega=args.angularspeed, alpha=args.angle, d=args.distance, debug=args.debug)
            controller.start_ros()
            controller.move()
        else:
            parser.print_help()
    except ros.ROSInterruptException:
        # We do this to avoid accidentally continuing to run code after the module is shut down
        pass