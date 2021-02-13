#! /usr/bin/env python

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import sys
import os
from scipy.optimize import curve_fit

# fit  y = func(x)
def fit(x, y, func):
    def exponential(x, a, b, c):
        return a * np.exp(-b * x) + c

    init = 0.001, -1, 0
    param, _ = curve_fit(exponential, x, y, init, maxfev=100000)
    return param


# Calculate the average for
root_path = os.path.expanduser(
    os.path.join(
        os.environ.get("ROBOTICS_PROJECT_DIR"), "Simulations/MediumWorld/Smart_Far"
    )
)
dir = os.listdir(root_path)
dir = [file for file in dir if file[20:] == "mines.txt"]
timesf = np.zeros((len(dir), 43))
min = 42
for i in range(len(dir)):
    mine_data = pd.read_csv(os.path.join(root_path, dir[i]), sep=";")
    time = mine_data["time"].values
    if time.shape[0] < min:
        min = time.shape[0]
    timesf[i, 1 : min + 1] = time[:min]
clearedsf = np.linspace(0, min, min + 1)
timesf = np.mean(timesf[:, : min + 1], axis=0)


root_path = os.path.expanduser(
    os.path.join(
        os.environ.get("ROBOTICS_PROJECT_DIR"), "Simulations/MediumWorld/Random_Roomba"
    )
)
dir = os.listdir(root_path)
dir = [file for file in dir if file[20:] == "mines.txt"]
timerr = np.zeros((len(dir), 43))
min = 42
for i in range(len(dir)):
    mine_data = pd.read_csv(os.path.join(root_path, dir[i]), sep=";")
    time = mine_data["time"].values
    if time.shape[0] < min:
        min = time.shape[0]
    timerr[i, 1 : min + 1] = time[:min]
clearedrr = np.linspace(0, min, min + 1)
timerr = np.mean(timerr[:, : min + 1], axis=0)


# function to fit
def exponential(x, a, b, c):
    return a * np.exp(-b * x) + c


# Create the plot
fig, ax = plt.subplots()
ax.set_ylabel("Average Time Until Detonation [s]")
ax.set_xlabel("number of mines")
ax.set_title("Average Time Until Detonation")

x = np.linspace(0, 42, 43)
a, b, c = fit(clearedsf, timesf, exponential)
print(f"Smart Far {exponential(42, a, b, c)}")
ax.plot(
    x,
    exponential(x, a, b, c),
    "-",
    label=f"{a:.2f} exp({-b:.2f}x){c:.2f}",
    color="blue",
    linewidth=2,
)
ax.plot(clearedsf, timesf, "o-", label="Smart Far", color="red")

a, b, c = fit(clearedrr, timerr, exponential)
print(f"Random Roomba {exponential(42, a, b, c)}")
ax.plot(
    x,
    exponential(x, a, b, c),
    "-",
    label=f"{a:.3f}exp({-b:.3f}x){c:.3f}",
    color="black",
    linewidth=2,
)
ax.plot(clearedrr, timerr, "o-", label="Random Roomba", color="green")


ax.legend()
plt.show()