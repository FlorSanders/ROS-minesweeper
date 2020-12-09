#!/usr/bin.env python

import rospy as ros
import tf
import sys
import numpy as np
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
        # Start a new figure to visualize our data
        plt.figure(figsize = (30,30))
        # Use the superclass initialization function
        super(SmartRoomba, self).__init__(**params)

    def start_ros(self):
        """
        Overwrite of the start_ros function, we still have things to add
        """
        # Call the start_ros function of the superclass
        super(SmartRoomba, self).start_ros()
        # Initialize extra variables we want to be able to use
        self.map_visited = np.zeros(self.map_dim, dtype=bool)
        self.map_visited_origin = self.map_origin
        self.position_on_map = None
        self.orientation_on_map = None
    
    def update_map_visited(self):
        """
        Updates the visited map:
        - Adds the current point to the visited map.
        - If the original map changes shape or origin, update the new map to reflect the same
        """
        # Add the current position on the map
        robot_radius = 0.25
        coordinates = np.indices(self.map_dim).transpose((2,1,0))
        self.map_visited[np.linalg.norm(np.array(coordinates) - np.array(self.position_on_map)) < robot_radius/self.map_res] = True

        # If the map itself has changed --> Apply corrections as well
        if self.map_visited.shape != self.map.shape or self.map_visited_origin != map_origin:
            self.printd("Map has changed!!")
            # Correct the size of the map
            ## TODO

            # Corrected the origin of the map
            self.map_visited_origin = self.map_origin
        

    def get_pose_on_map(self):
        """
        Does all the right transformations to find the position of the robot in the map
        """
        # Obtain the (transformed) position and rotation from the topics
        try:
            translation, rotation = self.transformer.lookupTransform('/map', '/base_link', ros.Time(self.map_time[0], self.map_time[1]))
            self.position_on_map = (np.array(translation[0:2]) - np.array(self.map_origin)) / self.map_res
            self.orientation_on_map = tf.transformations.euler_from_quaternion(rotation)
            return True
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # Something could go wrong in transformation
            return False
        ros.logerr(self.position_on_map)

    def plot_map(self):
        """
        Makes a visualization of the map
        """
        # Configure matplotlib
        plt.clf()
        ax = plt.gca()

        # Convert the map values to something more usable
        map_visualize = self.map.copy()
        map_visualize[map_visualize == -1] = 250
        map_visualize[self.map_visited] = 150

        # Show our current position and orientation
        ax.scatter(self.position_on_map[1], self.position_on_map[0], color='black', s=25)
        ax.arrow(self.orientation_on_map[1]*10, self.orientation_on_map[0]*10, color='grey')

        # Plot the map
        ax.matshow(map_visualize.T)
        ax.invert_yaxis()
        ax.invert_xaxis()

        # Display the result
        plt.pause(1e-3)
        
    def move(self):
        """
        Moving strategy for our smart roomba
        """

        # Run as long as we're not shut down
        while not ros.is_shutdown():
            # Obtain the robot position in the map
            success = self.get_pose_on_map()
            if not success:
                continue

            # Update the visited map
            self.update_map_visited()

            # Plot map
            self.plot_map()

            # Move randomly for now
            self.move_randomly(duration=5)

    def move_randomly(self, duration=30):
        """
        Function from the randomroomba class, we can revert to this behaviour if all map usage goes down.
        For a given {duration} in seconds, do the following:
        1. Moves straight until the sensor detects possible collision.
        2. Turns towards an unblocked direction and continues.
        """
        # 
        t_init = ros.get_time()
        # If these are available, move straight until blocked or unblock
        while ros.get_time() - t_init < duration and not ros.is_shutdown():
            blocked, ranges = self.scan_for_obstacles()
            
            if blocked:
                # Picking out the directions that are not actually blocked
                unblocked_directions = np.concatenate([np.linspace(0, np.pi, len(ranges)//2), np.linspace(-np.pi, 0, len(ranges)//2)])[ranges >= self.thresh]
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
    r = SmartRoomba()
    r.start_ros()
    r.move()
