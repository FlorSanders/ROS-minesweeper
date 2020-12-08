#!/usr/bin.env python

# Importing the require libraries
import rospy as ros
import time
import sys
import numpy as np
import argparse
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from my_python_package.robot_controller import RobotController

# Third "minesweeper strategy": random movement with obstacle avoidance
class SquareSpiral(RobotController):
    """
    Class implements a trajectory in which the robot drives in a square spiral, applying obstacle avoidance when needed
    """
    def __init__(self, **params):
        """
        Initialize the roomba.
        """
        # Use the superclass initialization function
        super(SquareSpiral, self).__init__(**params)

    def goForwardUnlessBlocked(self, it):
        """
        Goes forward for {it} times the default duration unless it detects an obstacle,
        in which case it turns away from the obstacle and drives to a new region.
        """
        self.printd(f"Current distance multiplier: {it}")
        # Go forward by the default amount, it times, unless blocked
        for _ in range(it):
            # Check if we're blocked
            blocked, ranges = self.scan_for_obstacles()
            if blocked:
                self.printd("Obstacle detected")
                # Reseting our distance multiplier
                self.it=1
                # Picking out the directions that are not actually blocked
                unblocked_directions = np.concatenate([np.linspace(0, np.pi, len(ranges)//2), np.linspace(-np.pi, 0, len(ranges)//2)])[ranges >= self.thresh]
                # Making a probability distribution such that directions without any obstructions are more likely to be chosen
                probs = np.exp(ranges[ranges >= self.thresh])
                try:
                    probs = probs/np.sum(probs)
                except:
                    sys.exit("The robot is trapped, no free angles are available")
                random_choice = np.random.choice(unblocked_directions, p=probs, size=1)
                # Turning to the direction of freedom
                self.turn_by(alpha=random_choice, omega=self.omega)
                # Recursive call to this function to drive us away from the current obstacle without running into a new one
                self.goForwardUnlessBlocked(random.randint(3, 7)) # Generates a number to deterinate how far it goes 
                # Break away from the loop
                break
            else:
                # Go forward 
                self.go_forward_for()


    def drawSquareSpiral(self, it):
        """
        Drive in a square spiral pattern
        """
        self.printd(f"Amount of iterations: {it}")
        # Go forward unless blocked
        self.goForwardUnlessBlocked(it)
        # First turn 90 degrees
        self.turn_by(np.pi/2)
        
    
    def move(self):
        """
        Moves in a square spiral pattern until it detects an object in the way, in which case it moves away from that object and starts over
        """
        # Wait until sensor readings are available
        while (self.position is None or self.lds_ranges is None) and not ros.is_shutdown():
            print('Sleeping...')
            self.rate.sleep()

        self.it = 0
        # If these are available, move straight until blocked or unblock
        while not ros.is_shutdown():
            # Draw square spiral function already takes care of unblocking itself, just calling it for increased distances
            self.it += 1
            self.drawSquareSpiral(self.it)

# Starting the robot
def main():
    r = SquareSpiral()
    r.start_ros()
    r.move()
