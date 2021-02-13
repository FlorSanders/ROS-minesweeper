#! /usr/bin/env python

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import sys
import os

root_path = os.path.expanduser(
    os.path.join(os.environ.get("ROBOTICS_PROJECT_DIR"), "Simulations/")
)


# ask for the desired world name
worlds = [world for world in os.listdir(root_path) if world[-5:] == "World"]


def get_strategy(root_path):
    print("Enter the number of the desired world:")
    for world in range(len(worlds)):
        print(f"\t{world} - {worlds[world]}")
    world = input("world number: ")
    world_name = worlds[int(world)]

    # ask for the desired strategy
    print("Enter the number of the desired strategy")
    strategies = os.listdir(root_path + world_name)
    for strategy in range(len(strategies)):
        print(f"\t{strategy} - {strategies[strategy]}")
    strategy = input("strategy number: ")
    strategy_name = strategies[int(strategy)]
    root_path = os.path.join(root_path, world_name, strategy_name)

    # calculate the inter mine detonation time
    ipt = []
    clear_time = []
    cleared_mines = []
    n_mines = [40, 50, 30]
    dir = os.listdir(root_path)

    # Loop over all the files
    for file in dir:
        if file[20:] == "mines.txt":
            mine_data = pd.read_csv(os.path.join(root_path, file), sep=";")
            inter_mine_time = mine_data["time"].values

            # Calculate the 40% clearance time
            cleared = mine_data["count"].values
            clearance_time = inter_mine_time[cleared >= 0.4 * n_mines[int(world)]]
            if not clearance_time.size:
                clear_time.append(1e10)
            else:
                clear_time.append(np.max(clearance_time))

            # Calculate the inter mine detonation time
            inter_mine_time = inter_mine_time[1:] - inter_mine_time[:-1]
            ipt.append(inter_mine_time)
            cleared_mines.append(np.max(cleared))

    return (
        strategy_name,
        world_name,
        np.hstack(ipt),
        np.hstack(cleared_mines),
        np.array(clearance_time),
    )


IMDT = []
IMDT.append(get_strategy(root_path))
while "y" == input("Add more strategies? [y/n]: "):
    IMDT.append(get_strategy(root_path))


imdt = []
strategies = []
ct = []
# print some statistics
for strategy_name, world_name, data, cleared_mines, clearance_time in IMDT:
    print(f"\n Strategy: {world_name} {strategy_name}")
    print(f"\t mean imdt: {np.mean(data)}")
    print(f"\t maximum imdt: {np.max(data)}")
    print(f"\t average cleared mines: {np.mean(cleared_mines)}")
    print(f"\t average 40% clearance time: {np.mean(clearance_time)}")
    imdt.append(data)
    ct.append(clearance_time)
    strategies.append(strategy_name + "\n" + world_name)


# Create a boxplot imdt
fig, ax = plt.subplots()
ax.set_ylabel("IMDT[s]")
ax.set_title("Inter Mine Detonation Time")
ax.boxplot(imdt, labels=strategies, showfliers=False, showmeans=True)
plt.show()

# Boxplot 40% clearance time
fig, ax = plt.subplots()
ax.set_ylabel("Time[s]")
ax.set_title("Time until 40 percent of the mines are detonated")
ax.boxplot(ct, labels=strategies, showfliers=False, showmeans=True)
plt.show()
