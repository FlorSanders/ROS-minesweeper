#!/usr/bin.env python

# Importing required libraries
import roslaunch
import rospkg
import rospy as ros
import time
import os
import argparse
import sys

def launch_simulation(strategy, environment, duration=1800, repetitions=10, delay=25):
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
        # Launch gazebo and wait a bit
        gazebo_launcher = roslaunch.parent.ROSLaunchParent(uuid, [gazebo_path])
        gazebo_launcher.start()
        time.sleep(delay) # Could be too short if launching LARGE worlds... Dunno

        # If we're using a "smart" strategy, launch slam and navigation as well
        if 'smart' in strategy:
            # Locate both slam and navigation packages
            try:
                slam_path = (f"{packager.get_path('turtlebot3_slam')}/launch/turtlebot3_slam.launch", 'open_rviz:=false')
            except:
                sys.exit("The SLAM package couldn't be found")
            try:
                navigation_path = f"{packager.get_path('turtlebot3_navigation')}/launch/turtlebot3_navigation.launch"
            except:
                sys.exit("The navigation package couldn't be found")
            # Launch both the navigation and slam parts
            slam_launcher = roslaunch.parent.ROSLaunchParent(uuid, [slam_path])
            slam_launcher.start()
            navigation_launcher = roslaunch.parent.ROSLaunchParent(uuid, [navigation_path])
            navigation_launcher.start()
            time.sleep(delay)

        # Launch the robot driver
        driver_launcher = roslaunch.parent.ROSLaunchParent(uuid, [driver_path])
        driver_launcher.start()
            
        # Sleep for the duration we want the simulation to run for
        ros.sleep(duration)
        
        # Shutdown all launchers and wait a couple of seconds
        driver_launcher.shutdown()
        gazebo_launcher.shutdown()
        if 'smart' in strategy:
            slam_launcher.shutdown()
            navigation_launcher.shutdown()
        time.sleep(delay)

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
    strategies = ['random_roomba', 'worm', 'square_spiral', 'smart_random', 'smart_near', 'smart_far']

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
        help="How long (in seconds) each simulation will run for (in simulated time).",
        nargs='?',
        default=1800,
        type=float
    )
    parser.add_argument('--delay',
        help="Delay (in seconds) between simulations and node launches.",
        nargs='?',
        default=25,
        type=float
    )
    parser.add_argument('--repetitions',
        help="The total amount of times the simulation should be repeated.",
        nargs='?',
        default=10,
        type=int
    )

    # Parsing the arguments
    args = parser.parse_args()

    # Launching simulation
    launch_simulation(args.strategy[0], args.environment[0], args.duration, args.repetitions, args.delay)