#! /usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import pandas as pd
import numpy as np
import argparse
import sys
import os

def plot_trajectory(world, filename, simulation_time, mines):
    #Create the circumference of the world
    if (world==3):
        edges = np.array([[0,0],[0,5],[6,5], [6,0],[0,0]])
        tree = np.array([2,2])
        
    elif  (world==4):
        edges = np.array([[0,0],[0,6],[7,6], [7,0],[0,0]])
        tree = np.array([5.5,4])
        cones_x = 3*np.ones(4)
        cones_y = np.array([1, 2, 3, 4,5])
    elif (world == 5):
        edges = np.array([[0,0],[-2,0], [-2,2.75],[0,2.75],[0,6],[7,6], [7,0],[0,0]])
        tree = np.array([5.5,4])
        cones_x = 2*np.ones(4)
        cones_y = np.array([1, 2.25, 3.5, 4.75])
    else:
        edges = np.array([[0,0],[0,5],[6,5], [6,0],[0,0]])

    #retrieve path to the log files
    root = os.environ.get("ROBOTICS_PROJECT_DIR")
    file_name_traj = filename

    #read the coordinates of the robot and corresponding time stamp
    coords = pd.read_csv(os.path.join(root,"src/minesweeper_package/log/", file_name_traj), sep=';')
    x = coords["robot_x"].values
    y = coords["robot_y"].values

    #duration in seconds of the trajectory
    time = coords["time"].values
    if simulation_time==-1:
        simulation_time = time[-1]

    #read the position of the mines
    if mines:
        file_name_mines = filename[:20] + "mines.txt"
        mine_coords = pd.read_csv(os.path.join(root,"src/minesweeper_package/log/", file_name_mines), sep=';')
        x_mine = mine_coords["mine_x"].values
        y_mine = mine_coords["mine_y"].values

        file_name_mines = filename[:20] + "distribution.txt"
        mine_distr = pd.read_csv(os.path.join(root,"src/minesweeper_package/log/", file_name_mines), sep=';')
        x_distr = mine_distr["mine_x"].values
        y_distr = mine_distr["mine_y"].values
        for m_x, m_y in zip(x_mine, y_mine):
            for d_x, d_y in zip(x_distr, y_distr):
                if np.sqrt( (m_x-d_x)**2 + (m_y-d_y)**2)  < 1e-5:
                    select = np.invert(np.logical_and(y_distr==d_y, x_distr==d_x))
                    x_distr = x_distr[select]
                    y_distr = y_distr[select]

    #create circle
    theta = np.linspace(0, 2*np.pi, 100)
    circle = np.array([np.cos(theta), np.sin(theta)])
    
    #creating the plot
    fig, ax = plt.subplots()
    ax.set_title("ROBOT TRAJECTORY")
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")

    #plot the trajectory
    ax.plot(x[time < simulation_time], y[time < simulation_time],c='blue',linewidth=0.5)

    #plot the circumference of the world
    ax.plot(edges[:,0],edges[:,1],c='black',linewidth=3)

    #plot the begin position of the robot
    ax.plot(x[0],y[0], 'X',c='r',linewidth=5)

    #plot the tree
    ax.add_patch(Circle(tree, 0.2,color='black'))

    #plot the cones
    if (world > 3):
        for x, y in zip(cones_x, cones_y):
            ax.add_patch(Circle((x, y), 0.15 ,color='black'))
    #plot the fountain
    if world==5:
        ax.add_patch(Circle((-0.5,1.25), 0.3,color='black'))

    #plot the positions of the mines
    if mines:
        ax.plot(x_mine,y_mine, "o", c='green')
        ax.plot(x_distr, y_distr, "o", c='red')
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

    parser.add_argument('--mines', 
        action="store_true",
        help="Indicate the positions of the mines",
    )


    # Parsing the arguments
    args = parser.parse_args()


    plot_trajectory(args.world[0],args.file_name[0], args.duration, args.mines)

    
