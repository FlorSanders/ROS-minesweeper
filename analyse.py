#! /usr/bin/env python

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import sys
import os

root_path = os.path.expanduser(os.path.join(os.environ.get("ROBOTICS_PROJECT_DIR"),"Simulations/"))


#ask for the desired world name
worlds = os.listdir(root_path)
def get_strategy(root_path):
    print("Enter the number of the desired world:")
    for world in range(len(worlds)):
        print(f"\t{world} - {worlds[world]}")
    world = input("world number: ")
    world_name = worlds[int(world)]

    #ask for the desired strategy
    print("Enter the number of the desired strategy")
    strategies = os.listdir(root_path + world_name)
    for strategy in range(len(strategies)):
        print(f"\t{strategy} - {strategies[strategy]}")
    strategy = input("strategy number: ")
    strategy_name = strategies[int(strategy)]
    root_path =  os.path.join( root_path ,world_name , strategy_name )

    #calculate the inter mine detonation time
    ipt = []
    cleared_mines = []
    dir = os.listdir(root_path)
    for file in dir:
        if file[20:] == "mines.txt":
            mine_data = pd.read_csv(os.path.join(root_path , file), sep=';')
            inter_mine_time = mine_data['time'].values
            cleared_mines.append(mine_data['count'].values)
            inter_mine_time = inter_mine_time[1:] - inter_mine_time[:-1]
            ipt.append(inter_mine_time)
            
    return  strategy_name, world_name, np.hstack(ipt), np.hstack(cleared_mines)

IMDT = []
IMDT.append(get_strategy(root_path))
while "y" == input("Add more strategies? [y/n]: "):
    IMDT.append(get_strategy(root_path))


imdt = []
strategies = []
#print some statistics 
for strategy_name, world_name, data, cleared_mines in IMDT:
    print(f"\ninter mine detonation time: {world_name} {strategy_name}")
    print(f"\t mean: {np.mean(data)}")
    print(f"\t maximum: {np.max(data)}")
    print(f"\t minimum: {np.min(data)}")
    print(f"\t average cleared mines: {np.mean(cleared_mines)}")
    imdt.append(data)
    strategies.append(strategy_name + "\n" + world_name) 


#Create a boxplot 
fig, ax = plt.subplots()
ax.set_ylabel("IMDT[s]")
ax.set_title("Inter Mine Detonation Time")
ax.boxplot(imdt, labels=strategies, showfliers=False)
plt.show()
