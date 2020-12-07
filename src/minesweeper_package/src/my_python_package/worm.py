#!/usr/bin.env python

# Importing the require libraries
import rospy as ros
import time
import sys
import numpy as np
import argparse

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from my_python_package.robot_controller import RobotController

# Second "minesweeper strategy": Goes to the top corner and and goes down covering each line
class Worm(RobotController):
    """
    Class implements a trajectory in which the robot goes to the top-right corner which then goes left, then goes to the next line.
    """

    def __init__(self, **params):
        """
        Initialize the robot.
        """
        # Use the superclass initialization function
        self.state = "Begin"
        super(Worm, self).__init__(**params)    
    
    def goForwardUntilBlocked(self):
        """
        Goes forward for {it} times the default duration unless it detects an obstacle,
        in which case it turns away from the obstacle and drives to a new region.
        """
        # Go forward, until we're blocked
        while(True):
            # Check if we're blocked
            blocked, ranges = self.scan_for_obstacles()
            if blocked:
                # Once we're blocked, return
                return
            else:
                # As long as we're not blocked, move forward
                self.go_forward_by(0.1)
    
    def move(self):
        """
        1. Wait until readings from the sensor are available.
        2. Drives to the right-top corner.  
        3. Loop
            3.1 Drives to the left wall.
            3.2. Drives to the right wall.
        """
        # Wait until sensor readings are available
        while (self.position is None or self.lds_ranges is None) and not ros.is_shutdown():
            self.printd('Sleeping...')
            self.rate.sleep()

        while not ros.is_shutdown():
            (blocked, _) = self.scan_for_obstacles()
            # Begin strategy
            if self.state == "Begin":
                self.goForwardUntilBlocked()
                self.turn_by(np.pi/2)
                self.state = "WormLoopLeft"
            # Just turned left
            elif self.state == "WormLoopLeft":
                self.goForwardUntilBlocked()
                self.turn_by(np.pi/2)
                # We shouldn't be blocked by this point
                if blocked:
                    self.turn_by(np.pi/4)
                else:
                    self.go_forward_by(0.1)
                    self.turn_by(np.pi/2)
                    self.state = "WormLoopRight"
            # Just turned right
            elif self.state == "WormLoopRight":
                self.goForwardUntilBlocked()
                self.turn_by(-np.pi/2)
                # We shouldn't be blocked by this point
                if blocked:
                    self.turn_by(np.pi/4)
                else:
                    self.go_forward_by(0.1)
                    self.turn_by(-np.pi/2)
                    self.state = "WormLoopLeft"

# Starting the robot
def main():
    r = Worm()
    r.start_ros()
    r.move()
