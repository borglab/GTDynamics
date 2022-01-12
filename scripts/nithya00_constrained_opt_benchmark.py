"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  nithya_yetong00_constrainedopt_benchmark.py
 * @brief Plot intermediate results for penalty method optimizer vs augmented lagrangian optimizer.
 * @author Nithya Jayakumar
 * @author Yetong Zhang
"""

import math
import numpy as np
import matplotlib.pyplot as plt 

# load optimization result data from file
def load_data(filename):
    data = []
    with open(filename) as data_file:
        lines = data_file.readlines()
        for line in lines:
            line = line.split()
            line = [float(data) for data in line]
            data.append(line)
    data = np.array(data)
    data_arranged = {}
    data_arranged["num_iters"] = data[:, 0]
    data_arranged["cumulative_iters"] = np.cumsum(data[:, 0])
    data_arranged["mu"] = data[:, 1]
    data_arranged["feasibility"] = data[:, 2]
    data_arranged["optimality"] = data[:, 3]
    return data_arranged

# create 2 plots: 
# plot1: penalty parameter vs. cumulative lm iterations
# plot2: feasiblity, optimality vs. cumulative lm iterations
def make_plot(data):
    # fig = plt.figure(figsize=(10, 10), dpi=160)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].set_yscale('log')
    axes[0].plot(data["cumulative_iters"], data["mu"])
    axes[0].set_xlabel("Cumulative L-M Iterations")
    axes[0].set_ylabel("Penalty Parameter mu")

    axes[1].set_xlabel("Cumulative L-M Iterations")
    axes[1].set_ylabel("Residual")
    axes[1].plot(data["cumulative_iters"], data["feasibility"], label = "Feasibility", color="b")
    axes[1].plot(data["cumulative_iters"], data["optimality"], label = "Optimality", color="r")
    axes[1].legend()

# Load data for penalty method optimizer.
data_penalty = load_data('build/scripts/penalty_data.txt')
data_augl = load_data('build/scripts/augl_data.txt')

make_plot(data_penalty)
make_plot(data_augl)
plt.show()
