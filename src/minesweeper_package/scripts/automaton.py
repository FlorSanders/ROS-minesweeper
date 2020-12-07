#!/usr/bin.env python

# Importing required libraries
import roslaunch
import rospkg
import rospy as ros
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

def launch_simulation(strategy, environment, duration=30, repetitions=10):
    """
    Launches {repetitions} simulations for a given {strategy}, taking {duration} minutes per simulation.
    Available strategies:
    - random_roomba
    - worm
    - square_spiral
    Available environments:
    - 0_BasicMinefield
    - 1_BasicMinefield_Small
    - 2_EnhancedMinefield_
    ...
    """

    # Create a ros "node" for this process, launching all the other stuff
    ros.init_node('automaton')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Obtaining the path of the launch files we want to initiate
    packager = rospkg.RosPack()
    gazebo_path = f"{packager.get_path('turtlebot3_gazebo')}/launch/{environment}.launch"
    driver_path = f"{packager.get_path('minesweeper_package')}/launch/{strategy}.launch"

    # Making launchers for the nodes
    gazebo_launcher = roslaunch.parent.ROSLaunchParent(uuid, [gazebo_path])
    driver_launcher = roslaunch.parent.ROSLaunchParent(uuid, [driver_path])

    for _ in range(repetitions):
        # Launch the gazebo world and wait a couple of seconds
        gazebo_launcher.start()
        ros.sleep(5)

        # Launch the robot and sleep until it's shutdown time
        driver_launcher.start()
        ros.sleep(60*duration)

        # Shutdown both launchers and wait a couple of seconds
        driver_launcher.shutdown()
        gazebo_launcher.shutdown()
        ros.sleep(10)
    




if __name__ == "__main__":
    launch_simulation('random_roomba', '0_BasicMinefield', duration=1, repetitions=2)