#!/usr/bin.env python

# Importing required libraries
import roslaunch
import rospkg
import rospy as ros
import time
import os
import argparse
import sys

def launch_simulation(strategy, environment, duration=30, repetitions=10):
    """
    Launches {repetitions} simulations for a given {strategy}, taking {duration} minutes per simulation.
    """

    for iteration in range(repetitions):
        # Create a ros "node" for this process, launching all the other stuff
        ros.init_node('automaton')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Obtaining the path of the launch files we want to initiate
        packager = rospkg.RosPack()
        try:
            gazebo_path = f"{packager.get_path('turtlebot3_gazebo')}/launch/{environment}.launch"
        except:
            sys.exit("The specified environment couldn't be found")
        try:
            driver_path = f"{packager.get_path('minesweeper_package')}/launch/{strategy}.launch"
        except:
            sys.exit("The specified strategy couldn't be found")
            
        print(f"\033[94mRunning simulation nr. {iteration}\033[0m")
        # Making launchers for the nodes
        gazebo_launcher = roslaunch.parent.ROSLaunchParent(uuid, [gazebo_path])
        driver_launcher = roslaunch.parent.ROSLaunchParent(uuid, [driver_path])
        # Launch the gazebo world and wait a couple of seconds
        gazebo_launcher.start()
        time.sleep(10) # Could be too short if launching LARGE worlds... Dunno

        # Launch the robot and sleep until it's shutdown time
        driver_launcher.start()
        ros.sleep(60*duration)

        # Shutdown both launchers and wait a couple of seconds
        driver_launcher.shutdown()
        gazebo_launcher.shutdown()
        time.sleep(5)

    # Shutting down the program with success
    sys.exit(0)

if __name__ == "__main__":
    # Obtain the options we have in terms of environments
    packager = rospkg.RosPack()
    environments_path = f"{packager.get_path('turtlebot3_gazebo')}/launch"
    files = os.listdir(environments_path)
    environments = sorted([name[:-7] for name in files])[0:6] # Only show the 6 ones we've created
    print(environments)

    # Obtain the options we have in terms of strategies
    strategies = ['random_roomba', 'worm', 'square_spiral']

    # Making use of a parser to get the program arguments
    parser = argparse.ArgumentParser(description = 'Automatically run simulations one after the other')
    parser.add_argument('strategy',
        help=f"Which minesweeping strategy to launch. Options: [{'/'.join(strategies)}]",
        nargs=1,
        type=str
    )
    parser.add_argument('environment',
        help=f"Which environment to launch in the gazebo simulator. Options: [{'/'.join(environments)}]",
        nargs=1,
        type=str
    )
    parser.add_argument('--duration',
        help="How long (in minutes) each simulation will run for (in simulated time).",
        nargs='?',
        default=30,
        type=float
    )
    parser.add_argument('--repetitions',
        help="The total amount of times the simulation should be repeated.",
        nargs='?',
        default=10,
        type=int
    )
    parser.add_argument('--debug',
        help="Should debug messages be printed?",
        action='store_true'
    )

    # Parsing the arguments
    args = parser.parse_args()

    # Print the arguments used if debug is enabled
    if args.debug:
        print(vars(args))

    # Launching simulation
    launch_simulation(args.strategy[0], args.environment[0], args.duration, args.repetitions)