import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import *

fig, axs = plt.subplots(3, 4, figsize=(14, 9))
# plt.tight_layout()
exp_names = ["soft", "penalty", "sqp", "elm", "ielm_sp"]
labels = ["soft", "penalty", "SQP", "CM-Opt", "CMC-Opt"]
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']

axs[0, 0].set_ylabel("cost")
axs[1, 0].set_ylabel("constraint violation")
axs[2, 0].set_ylabel("projected cost")

axs[0, 0].set_title("estimation")
axs[0, 1].set_title("block-pole")
axs[0, 2].set_title("cart-pole")
axs[0, 3].set_title("quadruped")

axs[2, 0].set_xlabel("iterations")
axs[2, 1].set_xlabel("iterations")
axs[2, 2].set_xlabel("iterations")
axs[2, 3].set_xlabel("iterations")

scenario_names = ["rss01_estimation", "rss03_cp_friction", "rss02_cp_limits", "rss04_quad_jump"]
for col_id, scenario_name in enumerate(scenario_names):
    scenario_folder = "data/" + scenario_name + "/"
    plot_optimization_progress(
        scenario_folder, exp_names, labels, colors, axs[0, col_id], axs[1, col_id], axs[2, col_id])

axs[0, 0].set_xlim(0, 15)
axs[1, 0].set_xlim(0, 15)
axs[2, 0].set_xlim(0, 15)
# axs[1, 0].set_yscale('symlog')

axs[0, 1].set_xlim(0, 100)
axs[1, 1].set_xlim(0, 100)
axs[2, 1].set_xlim(0, 100)
axs[1, 1].set_yscale('symlog')
# axs[0, 1].set_yscale('log')
# axs[1, 1].set_yscale('log')
# axs[2, 1].set_yscale('log')

axs[0, 2].set_xlim(0, 100)
axs[1, 2].set_xlim(0, 100)
axs[2, 2].set_xlim(0, 100)
# axs[2, 2].set_ylim(5, 20)
axs[0, 2].set_yscale('log')
axs[1, 2].set_yscale('symlog')
axs[2, 2].set_yscale('log')

axs[0, 3].set_xlim(0, 100)
axs[1, 3].set_xlim(0, 100)
axs[2, 3].set_xlim(0, 100)
axs[0, 3].set_yscale('log')
axs[1, 3].set_yscale('symlog')
axs[2, 3].set_yscale('log')

axs[0, 0].legend()

rows = 3
cols = 4
for i in range(rows):
    for j in range(cols):
        ax = axs[i, j]
        ax.title.set_fontsize(16)
        ax.xaxis.label.set_fontsize(14)
        ax.yaxis.label.set_fontsize(14)
        for item in ax.get_xticklabels() + ax.get_yticklabels():
            item.set_fontsize(12)

plt.savefig("figures/optimization_progress.pdf")
plt.show()
