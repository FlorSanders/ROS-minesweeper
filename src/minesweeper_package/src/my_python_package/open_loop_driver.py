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
from my_python_package.robot_controller import RobotController


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
        # Job is done -- Stopping the robot
        self.stop_robot()

# Starting the robot
def main():
    r = OpenLoopDriver()
    r.start_ros()
    r.move()
