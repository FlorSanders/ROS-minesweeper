#!/usr/bin.env python

# Importing required libraries
import roslaunch
import rospkg
import rospy as ros
from geometry_msgs.msg import PoseWithCovarianceStamped


# Mapping stage: Build a map of the unknown environment!
def mapping_stage(uuid):
    """
    Function that launches the correct nodes to build the map
    """

    ## Mapping stage 1: Quickly start building a map by driving away randomly (still using obstacle avoidance)

    # Finding the paths for the launch files we want to run
    packager = rospkg.RosPack()
    driver_path = f"{packager.get_path('minesweeper_package')}/launch/random_roomba.launch"
    slam_path = f"{packager.get_path('turtlebot3_slam')}/launch/turtlebot3_gmapping.launch"

    # Creating launchers for the nodes in this stage
    driver_launcher = roslaunch.parent.ROSLaunchParent(uuid = roslaunch.rlutil.get_or_generate_uuid(None, False), [driver_path])
    slam_launcher = roslaunch.parent.ROSLaunchParent(uuid = roslaunch.rlutil.get_or_generate_uuid(None, False), [slam_path])

    # Launching the nodes
    driver_launcher.start()
    slam_launcher.start()
    ros.sleep(5)
    driver_launcher.shutdown()

    ## Mapping stage 2: Use the explore package to complete the map

    # Finding the paths of the nodes needed for the exploration
    navigation_path = f"{packager.get_path('turtlebot3_navigation')}/launch/move_base.launch"
    explorer_path = f"{packager.get_path('explore_lite')}/launch/explore.launch"

    # Creating launchers for the nodes in this stage
    launcher2 = roslaunch.parent.ROSLaunchParent(uuid = roslaunch.rlutil.get_or_generate_uuid(None, False), [navigation_path, explorer_path])
    launcher2.start()

    ros.spin()



def launchnode():
    ## First launch gazebo and fake

    # Initializing a ros node for this script
    ros.init_node('NodeLauncher', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    packager = rospkg.RosPack()
    gazebo_path = f"{packager.get_path('turtlebot3_gazebo')}/launch/3_EnhancedMinefield_Small.launch"
    gazebo_launcher = roslaunch.parent.ROSLaunchParent(uuid = roslaunch.rlutil.get_or_generate_uuid(None, False), [gazebo_path])
    gazebo_launcher.start()

    ros.sleep(5)

    mapping_stage(uuid)


if __name__ == "__main__":
    launchnode()