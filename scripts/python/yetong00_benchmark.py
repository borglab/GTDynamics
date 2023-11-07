import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import *

scenario_name = "yetong07_e_quadruped_jump"
scenario_folder = "data/" + scenario_name + "/"

fig1, ax1 = plt.subplots()
plot_error_vs_constraint(ax1, scenario_folder)

fig2, axs2 = plt.subplots(2, 1, figsize=(8, 8))
plot_optimization_progress(axs2, scenario_folder)

fig3, axs3 = plt.subplots(3, 2, figsize=(8, 8))
plot_trajectory(axs3, scenario_folder)

plt.show()
