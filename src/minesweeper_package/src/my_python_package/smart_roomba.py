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

    def __init__(self, strategy, **params):
        """
        Initialize the roomba.
        """
        # Set the strategy
        if strategy in ['random', 'near', 'far']:
            self.strategy = strategy
            self.node_name = f'smart_{strategy}'
        else:
            sys.exit('The requested strategy is not available')
        # Use the superclass initialization function
        super(SmartRoomba, self).__init__(**params)

    def set_goal(self):
        """
        Pick one of the places that isn't visited yet.
        Depending on the strategy, it will give more weight to options that are closer or further away, or just pick randomly.
        """
        # Coordinates we will pick our goal from
        coordinates = np.indices(self.map_dim).transpose((2,1,0))

        # Condition that we cannot set goals too close to an occupied area (threshold of occupation at 50%)
        boundary_coordinates = coordinates[(self.map>50)*(self.map<=100)]
        not_near_boundaries = np.ones(self.map_dim, dtype=bool)
        for point in boundary_coordinates:
            not_near_boundaries[np.sum((coordinates - point)**2, -1) < (1.25*self.robot_radius/self.map_res)**2] = False
        
        # Filter on spaces we didn't visit yet and we know are not occupied
        options = coordinates[(self.map_visited == False)*(self.map >= 0)*(self.map <= 100)*not_near_boundaries]
        self.printd(f'options shape:{options.shape}')
        
        # This just picks a completely random options from the available ones
        if len(options) > 0:
            # Pick the goal on the map according to the desired strategy
            if self.strategy == 'random':
                self.printd('Random strategy')
                # All options have equal weights, there's no preference for distance
                choice = np.random.choice(range(len(options)))
            elif self.strategy == 'near':
                self.printd('Near strategy')
                # The closer the option, the better. Probability decreases inversely proportional to the square of the distance
                reciprocal_distances = 1 / (np.sum((options-self.position_on_map)**2,-1)+1e-6)
                probabilities = reciprocal_distances / np.sum(reciprocal_distances)
                # Pick the goal now
                choice = np.random.choice(range(len(options)), p=probabilities)
            elif self.strategy == 'far':
                self.printd('Far strategy')
                # The further the option, the better. Probability of getting chosen increases with the square of the distance
                distances = np.sum((options-self.position_on_map)**2,-1)
                probabilities = distances / np.sum(distances)
                # Pick the goal now
                choice = np.random.choice(range(len(options)), p=probabilities)
            # Set the actual goal
            self.goal_on_map = options[choice]
            # Compute the best orientation at the goal position considering our current position
            self.set_goal_angle()

    def move(self):
        """
        Moving strategy for our smart roomba
        """
        # Run as long as we're not shut down
        while not ros.is_shutdown():
            
            if not np.any(self.map_visited == None):
                # Pick a new goal to drive to
                self.printd('Set goal...')
                self.set_goal()

                # Plot the map
                self.printd('Show map...')
                self.__plot_map()

                # Send our goal to the server
                result = self.send_goal()
                if result:
                    self.printd('Goal execution finished...')
            else:
                self.printd('Waiting for valid map data...')
                self.rate.sleep()

    def __plot_map(self):
        """
        Updates the visualization on the map if the controller signals there's something new to be visualized
        This plotting function can only be run in the main loop
        """
        if self.map_update:
            self.map_update = False
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

            # Show our current goal if it's available
            if not np.any(self.goal_on_map == None):
                ax.scatter(self.goal_on_map[1], self.goal_on_map[0], color='white', s=25)
                ax.arrow(self.goal_on_map[1], self.goal_on_map[0], np.sin(self.goal_orientation_on_map)*10, np.cos(self.goal_orientation_on_map)*10, color='green')

            # Plot the map
            ax.matshow(map_visualize.T)
            ax.invert_yaxis()
            ax.invert_xaxis()

            # Actually show the map
            plt.pause(1e-3)

# Starting the robot
def main(strategy):
    r = SmartRoomba(strategy)
    r.start_ros()
    r.move()
