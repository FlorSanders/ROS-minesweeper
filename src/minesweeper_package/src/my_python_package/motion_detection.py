#!/usr/bin/env python

import time
import rospy as ros
import numpy as np
import matplotlib.pyplot as plt

from my_python_package.abstract_turtle import AbstractTurtle


class MotionDetection(AbstractTurtle):
    """
    This class opens a graphic interface, showing real-time
    representations of the LiDAR scan and a naive motion-detection
    algorithm that uses LiDAR data.
    """

    def __init__(self):
        super(MotionDetection, self).__init__()

        # placeholders for keeping track of the LiDAR scan, point cloud,
        # and time at the previous timestep (t-1)
        self.scan_tm1, self.point_cloud_tm1, self.tm1 = None, None, None

    def run(self):
        self._startup()

        while not ros.is_shutdown():
            plt.clf() # clear plot

            scan = self._process_lidar_scan()
            motion = self._compute_motion()

            self._polar_plot(scan, 121, 2.5, "LiDAR")
            self._polar_plot(motion, 122, 1.99, "Motion")
           
            # show plot in non-blocking manner
            plt.pause(0.001)
            self._sleep()

    def _startup(self):
        # Initialize variables
        self.scan_tm1 = self._process_lidar_scan()
        self.point_cloud_tm1 = self._scan2cloud(self.scan_tm1)
        self.tm1 = time.time()

        # Initialize plot
        plt.figure(figsize=(10.24, 5.12))
        plt.gcf().canvas.set_window_title("Turtlebot3 Motion Detection")

    def _process_lidar_scan(self):
        # get lidar scan from topic
        scan = np.array(self.scan_msg.ranges)

        # if the lidar sees nothing in some direction it returns a range
        # of 0. In gazebo simulation, if the lidar sees nothing it 
        # returns a range of infinite. For our plot we want to display 
        # the max range to indicate there is nothing in that direction. 
        scan[scan == 0.] = 2.5
        scan[scan == np.inf] = 2.5

        return scan

    def _compute_motion(self):
        """
        For motion detection, we compare point clouds computed from the
        lidar scans at time t and t-1. Considering how the lidar takes
        measurements, think about why this method is better for motion
        detection than simply comparing the lidar scans themselves.
        """

        # Get lidar scan from topic
        scan = np.array(self.scan_msg.ranges)
        scan[scan == np.inf] = 0. 

        # Estimate distances between points in the point clouds at time
        # t and t-1.
        point_cloud_t = self._scan2cloud(scan)
        distance_matrix = self._euclidean_distance_matrix(
            point_cloud_t, self.point_cloud_tm1)
        # We define motion as smallest distance from a point at time t 
        # to any point in the point cloud at t-1
        motion = np.min(distance_matrix, axis=1)

        # Filter invalid measurements
        motion[scan == 0.] = 0.
        motion[self.scan_tm1 == 0.] = 0.

        # Update point cloud for next timestep
        self.scan_tm1 = scan
        self.point_cloud_tm1 = point_cloud_t

        # Make the plot
        return motion / self.pub_rate


    def _polar_plot(self, values: np.ndarray, subplot_idx: int, 
                    ylim: float, title=None):
        """
        Make a polar plot, also known as a star, spider, or radar plot.
        For further explanation:
        https://medium.com/python-in-plain-english/radar-chart-basics-with-pythons-matplotlib-ba9e002ddbcd
        """

        # compute angles corresponding with indices in values
        angles = np.arange(0, 2 * np.pi, 2 * np.pi / len(values))

        # repeat first value to close the circle
        values = np.append(values, values[0])
        angles = np.append(angles, angles[0])

        # make plot with matplotlib
        # For documentation, see:
        # https://matplotlib.org/3.3.2/api/_as_gen/matplotlib.pyplot.plot.html
        ax = plt.subplot(subplot_idx, polar=True)
        plt.polar(angles, values, linewidth=0)
        plt.fill(angles, values, alpha=0.3)
        plt.ylim(0., ylim)
        ax.set_rlabel_position(0)
        plt.yticks(list(range(1, int(ylim) + 1)), color='grey', size=10)
        if title is not None:
            ax.set_title(str(title))

    def _sleep(self):
        """
        Plotting and computing distance matrices can be expensive
        procedures. Therefore, we don't want to use 
        "time.sleep(self.pub_rate)" as it does not account for the time
        lost on these procedures
        """
        time.sleep(max(0., self.tm1 + self.pub_rate - time.time()))
        self.tm1 = time.time()

    def _scan2cloud(self, scan: np.ndarray):
        """
        The /scan topic gives an array containing 360 range readings
        between 0 and 2*pi. This function converts this array to a point
        cloud centered on the LiDAR, such that each range reading is
        expressed as a point (x,y)
        """
        angles_rad = np.arange(0, 2 * np.pi, 2 * np.pi / scan.shape[0])

        # convert angles to projections on the unit circle
        angles_cos = np.cos(angles_rad)
        angles_sin = np.sin(angles_rad)

        # multiply these points by their corresponding range reading
        return np.stack((angles_cos * scan, angles_sin * scan), axis=1)

    def _euclidean_distance_matrix(self, A: np.ndarray, B: np.ndarray):
        """
        This is a numpy-efficient way to compute a distance matrix
        between two arrays. For explanation:
        https://medium.com/swlh/euclidean-distance-matrix-4c3e1378d87f
        """
        p1 = np.sum(A ** 2, axis=1)[:, np.newaxis]
        p2 = np.sum(B ** 2, axis=1)
        p3 = -2 * np.dot(A, B.T)
        P = p1 + p2 + p3
        # rounding errors can cause very small negative values that    
        # should be 0.
        P[P < 0] = 0 
        return np.sqrt(P)


def main():
    r = MotionDetection()
    r.start_ros()
    r.run()