#!/usr/bin.env python

import rospy as ros
import tf
import sys
import numpy as np
import time
import matplotlib.pyplot as plt

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
            
            # Plot the map
            self.__plot_map()

            # Go to sleep again
            self.printd('Sleeping again...')
            self.rate.sleep()

    def __plot_map(self):
        """
        Updates the visualization on the map if the controller signals there's something new to be visualized
        This plotting function can only be run in the main loop
        """
        if self.map_update:
            self.map_update = False
            self.printd('Plotting map...')
            # Configure matplotlib
            plt.clf()
            ax = plt.gca()

            # Convert the map values to something more usable
            map_visualize = self.map.copy()
            map_visualize[map_visualize == -1] = 250
            map_visualize[self.map_visited] = 150

            # Show our current position and orientation
            ax.scatter(self.position_on_map[1], self.position_on_map[0], color='black', s=25)
            ax.arrow(self.position_on_map[1], self.position_on_map[0], np.sin(self.orientation_on_map)*10, np.cos(self.orientation_on_map)*10, color='grey')

            # Plot the map
            ax.matshow(map_visualize.T)
            ax.invert_yaxis()
            ax.invert_xaxis()

            # Actually show the map
            plt.pause(1e-3)

# Starting the robot
def main():
    r = SmartRoomba()
    r.start_ros()
    r.move()
