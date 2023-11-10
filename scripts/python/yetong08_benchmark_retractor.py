import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import load_csv

exp_names = ["hierarchical_none", "hierarchical_a", "hierarchical_T",
             "hierarchical_metric", "barrier_none", "barrier_a", "barrier_T", "barrier_metric"]

figure, axes = plt.subplots(1, 2, figsize=(12, 5))

for exp_name in exp_names:
    state_file_path = "data/quadruped_ground_air_" + exp_name + "_states.csv"
    trial_file_path = "data/quadruped_ground_air_" + exp_name + "_trials.csv"

    states = load_csv(state_file_path)
    trials = load_csv(trial_file_path)

    axes[0].plot(states["iterations"], states["error"], label=exp_name)
    axes[1].plot(states["iterations"], states["lambda"], label=exp_name)

axes[0].set_yscale('log')
axes[0].set_ylabel('error')
axes[1].set_yscale('log')
axes[1].set_ylabel('lambda')
axes[1].legend()
plt.show()
