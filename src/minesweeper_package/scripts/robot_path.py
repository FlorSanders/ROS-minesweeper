#! /usr/bin/env python

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import sys
import os

def plot_trajectory(world, filename, simulation_time):
    #Enter the number of the world
    world = 4

    if (world==3):
        edges = np.array([[0,0],[0,5],[6,5], [6,0],[0,0]])
    elif  (world==4):
        edges = np.array([[0,0],[0,6],[7,6], [7,0],[0,0]])
    elif (world == 5):
        edges = np.array([[0,0],[-2,0], [-2,2],[0,2],[0,6],[7,6], [7,0],[0,0]])
    else:
        edges = np.array([[0,0],[0,5],[6,5], [6,0],[0,0]])

    #retrieve path to the log files
    root = os.environ.get("ROBOTICS_PROJECT_DIR")
    file_name = "test.csv"

    #read the coordinates of the robot and corresponding time stamp
    coords = pd.read_csv(os.path.join(root,"src/minesweeper_package/log/", file_name), sep=';')
    x = coords["robot_x"].values
    y = coords["robot_y"].values

    #duration in seconds of the trajectory
    time = coords["time"].values
    if simulation_time==-1:
        simulation_time = time[-1]



    #creating the plot
    fig, ax = plt.subplots()
    ax.set_title("ROBOT TRAJECTORY")
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.plot(x[time < simulation_time], y[time < simulation_time],c='black',linewidth=0.5)
    ax.plot(edges[:,0],edges[:,1],c='r',linewidth=3)
    ax.plot(x[0],y[0], 'X',c='r',linewidth=5)
    plt.show()

 # Making use of a parser to get the program arguments
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'plots the trajectory of the robot')
    parser.add_argument('world',
        help=f"Number of the world used during the simulation, options: [{'/'.join(['3', '4', '5'])}]",
        nargs=1,
        type=int
    )
    parser.add_argument('file_name',
        help="name of the file containing data for the robot trajectory",
        nargs=1,
        type=str
    )
    parser.add_argument('--duration',
        help="time in seconds of the simulation",
        nargs=1,
        default=-1,
        type=int
    )


    # Parsing the arguments
    args = parser.parse_args()


    plot_trajectory(args.world,args.file_name, args.duration)

    
