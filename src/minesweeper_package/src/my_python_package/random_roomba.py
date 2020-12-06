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

# Starting the robot
def main():
    r = RandomRoomba()
    r.start_ros()
    r.move()
