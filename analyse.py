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
dir = os.listdir(root_path)
for file in dir:
    if file[20:] == "mines.txt":
        mine_data = pd.read_csv(os.path.join(root_path , file), sep=';')
        inter_mine_time = mine_data['time'].values
        inter_mine_time = inter_mine_time[1:] - inter_mine_time[:1]
        ipt.append(inter_mine_time)


#print some statistics
x = np.hstack(ipt)
print("\ninter mine detonation time:")
print(f"\t mean: {np.mean(x)}")
print(f"\t maximum: {np.max(x)}")
print(f"\t minimum: {np.min(x)}")

#Create a boxplot 
fig, ax = plt.subplots()
ax.boxplot(x)
ax.set_ylabel("IMDT[s]")
ax.set_title("Inter Mine Detonation Time")
plt.show()
