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
        self.previous_on_map = None
        self.orientation_on_map = None
        self.goal_on_map = None
        self.time_plot = 0
    
    def update_map_visited(self):
        """
        Updates the visited map:
        - Adds the current point to the visited map.
        - If the original map changes shape or origin, update the new map to reflect the same
        """
        # Add the current position on the map
        robot_radius = 0.1
        coordinates = np.indices(self.map_dim).transpose((2,1,0))
        # Add the track we visited to the map
        if np.any(self.previous_on_map == None):
            self.map_visited[np.sum((coordinates - self.position_on_map)**2, -1) < (robot_radius/self.map_res)**2] = True
        else:
            n_points = 10
            direction = (self.position_on_map - self.previous_on_map)/(n_points-1)
            for i in range(n_points):
                point = self.previous_on_map + i*direction
                self.map_visited[np.sum((coordinates - point)**2, -1) < (robot_radius/self.map_res)**2] = True

        # If the map itself has changed --> Apply corrections as well
        if self.map_visited.shape != self.map.shape or np.any(self.map_visited_origin != self.map_origin):
            self.printd("Map has changed!!")
            # Correct the size of the map
            ## TODO --> Necessary?

            # Corrected the origin of the map
            self.map_visited_origin = self.map_origin
        
    def get_pose_on_map(self):
        """
        Does all the right transformations to find the position of the robot in the map
        """
        # Obtain the (transformed) position and rotation from the topics
        try:
            translation, rotation = self.transformer.lookupTransform('/map', '/base_link', ros.Time(self.map_time[0], self.map_time[1]))
            self.previous_on_map = self.position_on_map
            self.position_on_map = (np.array(translation[0:2]) - self.map_origin) / self.map_res
            _,_,self.orientation_on_map = tf.transformations.euler_from_quaternion(rotation)
            return True
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # Something could go wrong in transformation
            return False

    def plot_map(self):
        """
        Makes a visualization of the map
        """
        tau = 15
        now = time.time()
        # Don't try to plot every iteration
        if now - self.time_plot > tau:
            # Reset plot time
            self.time_plot = now

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

            # Show our current goal and ideal orientation for our robot if available
            if not np.any(self.goal_on_map == None):
                ax.scatter(self.goal_on_map[1], self.goal_on_map[0], color='white',s=25)
                alpha = self.get_goal_angle()
                ax.arrow(self.position_on_map[1], self.position_on_map[0], np.sin(alpha)*10, np.cos(alpha)*10, color='green')

            # Plot the map
            ax.matshow(map_visualize.T)
            ax.invert_yaxis()
            ax.invert_xaxis()

            # Actually show the map
            plt.pause(tau)

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

    def goal_reached(self):
        """
        Returns boolean value which indicates whether we should set a new goal.
        This happens when either no goal is set yet, the goal is reached or (maybe later) if we already spent too long trying to reach the current goal.
        """
        if not np.any(self.goal_on_map == None):
            reached = self.map_visited[self.goal_on_map[0],self.goal_on_map[1]]
            if reached:
                self.printd('We reached our goal!!')
        else:
            reached = True
        return reached

    def get_goal_angle(self):
        """
        Returns the orientation the robot should have to be turned towards the goal
        """
        vector = self.goal_on_map - self.position_on_map
        alpha = np.arctan(vector[1]/vector[0])
        if vector[0] < 0:
            self.printd("Yellow")
            alpha = np.pi+alpha
        return alpha

    def go_forward_unless_blocked(self):
        """
        Go forward, unless we're blocked, then call the unlock function
        """
        # Check if we're blocked
        blocked, ranges = self.scan_for_obstacles()
        if blocked:
            # Unblock yourself
            self.unblock(ranges)
        else:
            # Drive forward
            self.go_forward_for()

    def unblock(self, ranges):
        """
        Naive way to unblock our route: just see which way is free and turn that way
        Also move a little forward
        """
        self.printd('Unblocking...')
        # Picking out the directions that are not actually blocked
        unblocked_directions = np.concatenate([np.linspace(0, np.pi, len(ranges)//2), np.linspace(-np.pi, 0, len(ranges)//2)])[ranges >= self.thresh]
        # Making a probability distribution such that directions without any obstructions are more likely to be chosen
        probs = np.exp(ranges[ranges >= self.thresh])
        try:
            probs = probs/np.sum(probs)
        except:
            sys.exit("The robot is trapped, no free angles are available")
        random_choice = np.random.choice(unblocked_directions, p=probs, size=1)
        # Drive backwards to make sure we have space to turn
        self.go_forward_by(d=-0.15, v=-0.5)
        # Make the preplanned turn
        self.turn_by(alpha=random_choice, omega=self.omega)
        # Go forward a little unless blocked again
        self.go_forward_unless_blocked()

    def move_to_goal(self):
        """
        Finds and executes a plan to move towards the goal we set.
        If no goal is available it uses the "dumb" roomba strategy as a fallback.
        """
        if not np.any(self.goal_on_map == None):
            ## 1. Turn so that our orientation is pointed towards our goal
            # Obtain the orientation we should be in
            alpha = self.get_goal_angle()
            # Check if we're approximately aligned with our goal
            self.printd((self.orientation_on_map, alpha))
            if abs(self.orientation_on_map - alpha) < np.pi/8:
                # If so: move forward
                self.printd('Moving...')
                self.go_forward_unless_blocked()
            else:
                # Turn to align ourself
                self.printd('Turning...')
                self.turn_for()
        else:
            # If no goal is available, just move randomly
            self.go_forward_unless_blocked()
        
    def move(self):
        """
        Moving strategy for our smart roomba
        """
        # Run as long as we're not shut down
        while not ros.is_shutdown():       
            # Obtain the robot position and orientation in the map
            success = self.get_pose_on_map()
            if not success:
                # If we didn't manage to get our pose, skip this iteration
                continue

            # Update the visited map
            self.update_map_visited()

            # Check if we reached our set goal yet
            if self.goal_reached():
                # Set a new goal
                self.set_goal()
            else:
                # Move to our goal
                self.move_to_goal()

            # Plot map
            # self.plot_map()
            
            # Sleep
            self.rate.sleep()

# Starting the robot
def main():
    r = SmartRoomba()
    r.start_ros()
    r.move()
