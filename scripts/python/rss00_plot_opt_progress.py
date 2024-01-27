import numpy as np
import matplotlib.pyplot as plt
from yetong00_utils import *

fig, axs = plt.subplots(3, 4, figsize=(14, 9))
# plt.tight_layout()
exp_names = ["soft", "penalty", "sqp", "elm", "iegd", "ielm_sp", "ielm_cr"]

axs[0, 0].set_ylabel("cost")
axs[1, 0].set_ylabel("constraint violation")
axs[2, 0].set_ylabel("projected cost")

axs[0, 0].set_title("estimation")
axs[0, 1].set_title("cart-pole")
axs[0, 2].set_title("block-pole")
axs[0, 3].set_title("quadruped")

axs[2, 0].set_xlabel("iterations")
axs[2, 1].set_xlabel("iterations")
axs[2, 2].set_xlabel("iterations")
axs[2, 3].set_xlabel("iterations")

scenario_names = ["rss01_estimation", "rss02_cp_limits"]
for col_id, scenario_name in enumerate(scenario_names):
    scenario_folder = "data/" + scenario_name + "/"
    plot_optimization_progress(
        scenario_folder, exp_names, axs[0, col_id], axs[1, col_id], axs[2, col_id])

axs[0, 0].set_xlim(0, 15)
axs[1, 0].set_xlim(0, 15)
axs[2, 0].set_xlim(0, 15)

axs[0, 1].set_xlim(0, 60)
axs[1, 1].set_xlim(0, 60)
axs[2, 1].set_xlim(0, 60)
axs[0, 1].set_yscale('log')
# axs[1, 1].set_yscale('log')
axs[2, 1].set_yscale('log')

axs[0, 0].legend()
plt.savefig("figures/optimization_progress.png")
plt.show()
