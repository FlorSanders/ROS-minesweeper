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

    def __init___(self, **params):
        """
        Initialize the roomba.
        """
        # Use the superclass initialization function
        super(Worm, self).__init__(**params)
        self.states = ["Begin", "TopRightCorner", "WormLoopLeft", "WormLoopRight" ]
        self.state = states[0]
    
    def move(self):
        """
        1. Wait until readings from the sensor are available.
        2. Drives to the right-top corner.  
        3. Loop
            3.1 Drives to the left wall.
            3.2. Drives to the right wall.
        """
        def goDown():
            if self.state == self.states[2]:
                self.turn_by(np.pi/2, 0.5)
                self.go_forward_by(self.d, self.v)
                self.turn_by(-np.pi/2, 0.5)
            elif self.state == self.states[3]:
                self.turn_by(-np.pi/2, 0.5)
                self.go_forward_by(self.d, self.v)
                self.turn_by(np.pi/2, 0.5)
    
        # Wait until sensor readings are available
        while (self.position is None or self.lds_ranges is None) and not ros.is_shutdown():
            print('Sleeping...')
            self.rate.sleep()

        # If these are available, move straight until blocked or unblock
        while not ros.is_shutdown():
            (blocked, _) = self.scan_for_obstacles()
            if self.state == self.states[0]:        # rotate 45 degrees and go forward
                self.turn_by(np.pi/4, self.v)
                self.state = self.states[1]

            elif self.state == self.states[1]:  # keep going forward until blocked
                if blocked:
                    self.turn_by(-135*np.pi/180, self.v)
                    self.state = self.states[2]
                else:     
                    self.go_forward_by(self.d, self.v)
            elif self.state == self.states[2]:  # rotate 180 degrees and go forward
                self.go_forward_by(self.d, self.v)
                if blocked:
                    goDown()
                    self.state = self.states[3]
            elif self.state == self.states[3]:  # rotate 180 degrees and go forward
                self.go_forward_by(self.d, self.v)
                if blocked:              
                    goDown()
                    self.state = self.states[2]

# Starting the robot
def main():
    r = Worm()
    r.start_ros()
    r.move()
