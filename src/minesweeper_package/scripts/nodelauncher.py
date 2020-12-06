#!/usr/bin.env python

# Importing required libraries
import roslaunch
import rospy as ros

package = 'minesweeper_package'
executable = 'random_roomba'

def launchnode():
    # Launching a node
    node = roslaunch.core.Node(package, executable)

    launcher = roslaunch.scriptapi.ROSLaunch()
    launcher.start()

    process = launcher.launch(node)
    print(process.is_alive())
    ros.sleep(30)
    process.stop()
    ros.sleep(10)

    print("Shutting down the script")


if __name__ == "__main__":
    launchnode()