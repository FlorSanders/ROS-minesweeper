#!/usr/bin.env python

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
        ranges[np.isinf(ranges)] = 4.

        # Select ranges in the direction we are driving
        relevant_ranges = np.concatenate((ranges[-12:], ranges[:12])) # Not too sure about these indices
        blocked = np.any(relevant_ranges < self.thresh)
        
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
                # Picking out the directions that are not actually blocked
                unblocked_directions = np.concatenate([np.linspace(0, np.pi, len(ranges)/2), np.linspace(-np.pi, 0, len(ranges)/2)])[ranges >= self.thresh]
                # Making a probability distribution such that directions without any obstructions are more likely to be chosen
                probs = np.exp(ranges[ranges >= self.thresh])
                try:
                    probs = probs/np.sum(probs)
                except:
                    sys.exit("The robot is trapped, no free angles are available")
                random_choice = np.random.choice(unblocked_directions, p=probs, size=1)
                self.turn_by(alpha=random_choice, omega=self.omega)
            else:
                self.go_forward_for(tau=self.tau, v=self.v)
def main():
    r = RandomRoomba()
    r.start_ros()
    r.move()
