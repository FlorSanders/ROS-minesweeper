#!/usr/bin.env python

import rospy as ros
import tf
import sys
import numpy as np
import matplotlib.pyplot as plt
import time

from my_python_package.robot_slam_controller import RobotSlamController

# First "minesweeper strategy": random movement with obstacle avoidance
class SmartRoomba(RobotSlamController):
    """
    This class makes use of the map data provided by the gmapping topic.
    The goal is to make smart decisions of where to go to, making sure to visit every piece of the map
    """

    def __init__(self, **params):
        """
        Initialize the roomba.
        """
        # Use the superclass initialization function
        super(SmartRoomba, self).__init__(**params)

    def set_goal(self):
        """
        Pick one of the places that isn't visited yet, hopefully in a smart way
        """
        # Coordinates we will pick our goal from
        coordinates = np.indices(self.map_dim).transpose((2,1,0))
        # Filter on spaces we didn't visit yet and we know are not occupied
        options = coordinates[(self.map_visited == False)*(self.map >= 0)*(self.map <= 100)]
        # Just pick a totally random option for now --> Create a costmap later
        self.goal_on_map = options[np.random.choice(range(len(options)))]

    def move(self):
        """
        Moving strategy for our smart roomba
        """
        # Run as long as we're not shut down
        while not ros.is_shutdown():       
            # Simply sleep the time away for now
            self.rate.sleep()

# Starting the robot
def main():
    r = SmartRoomba()
    r.start_ros()
    r.move()
