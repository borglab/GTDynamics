import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import load_data, draw_half_sphere


def plot_summary(axs, summary, exp_name, color):
    axs[0].plot(summary[:,0], summary[:,1], label=exp_name, color=color)
    axs[1].plot(summary[:,0], summary[:,2], label=exp_name + "_e", color=color)
    axs[1].plot(summary[:,0], summary[:,3], label=exp_name + "_i", color=color, linewidth=0.4)


exp_names = ["soft", "barrier", "lm", "gd"]
colors = ["r", 'g', "b", "c"]
fig, axs= plt.subplots(2, 1, figsize=(8,8))

for exp_name, color in zip(exp_names, colors):
    folder = "results/pole_" + exp_name + "/"
    summary = load_data(folder + "summary.txt")

    plot_summary(axs, summary, exp_name, color)

axs[0].legend()
axs[1].legend()
axs[1].set_xlabel("iterations")
axs[1].set_ylabel("constraint violation")
axs[0].set_ylabel("cost")
axs[1].set_yscale("log")
plt.savefig("results/cp_loss.pdf")
plt.show()
