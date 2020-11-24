#!/usr/bin/env python

import numpy as np
import time
import rospy as ros
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from my_python_package.abstract_turtle import AbstractTurtle

class ObstacleAvoidance(AbstractTurtle):
    """
    This class controls a turtleBot to drive a random trajectory while
    avoiding obstacles. The control pattern is very simple: the 
    turtleBot drives forward until its LiDAR reads a range of less than
    0.5 meters on a narrow span of its 360 degree view, right in front
    of the robot. The turtleBot then turns to a new direction sampled at
    random.
    """
    def __init__(self):
        super(ObstacleAvoidance, self).__init__()

    def run(self):
        """ main loop of the program """
        while not ros.is_shutdown():
            if self._scan4obstacles():
                # if trajectory is blocked: turn in random direction
                self._turn(np.random.uniform(-np.pi, np.pi))
            else:
                # else: send "move forward" command
                self._move()

    def _move(self, speed=0.5):
        """ 
        send a single move command to the turtlebot. We can not send
        commands over a period of time here, like we did in the
        open-loop and semi-closed-loop examples in week 2. If we did
        that, we would not be able to scan for obstacles while driving
        forward.
        """
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0
        self.vel_ros_pub(msg)
        time.sleep(self.pub_rate)

    def _turn(self, a, ang_speed=0.4):
        # store angle at previous timestep
        angle_tm1 = self._get_z_rotation(
            self.odom_msg.pose.pose.orientation)
        turned = 0.

        # Turn until target angle is reached
        while not ros.is_shutdown():
            # keep track how far robot has turned
            angle_t = self._get_z_rotation(
                self.odom_msg.pose.pose.orientation)
            turned += (angle_t - angle_tm1 + np.pi) % \
                      (2 * np.pi) - np.pi
            angle_tm1 = angle_t

            # stop once target angle has been exceeded
            if turned < a < 0 or a == 0 or turned > a > 0:
                break

            msg = Twist()

            # positive angular speed -> turn counter-clockwise
            msg.angular.z = ang_speed if a > 0 else -ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def _scan4obstacles(self):
        ranges = np.array(self.scan_msg.ranges)
        ranges_front = np.concatenate((ranges[-12:], ranges[:12]))
        return np.any(np.logical_and(ranges_front > 0., 
                                     ranges_front < 0.5))

    def _get_z_rotation(self, orientation):
        

        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, 
             orientation.z, orientation.w])
        return yaw

def main():
    r = ObstacleAvoidance()
    r.start_ros()
    r.run()