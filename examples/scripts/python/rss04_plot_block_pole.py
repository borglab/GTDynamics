import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches as patches
from yetong00_utils import load_csv


def plot_state(ax, q):
    colormap = mpl.colormaps["jet"]
    for i in range(0, len(q), 1):
        ax.plot([0, np.sin(q[i])], [0, -np.cos(q[i])], color=colormap((len(q)-i)/len(q)))


folder = "data/rss03_cp_friction/"
exp_name= "ielm"

init_data = load_csv(folder + "init_traj.csv")
manopt_data = load_csv(folder + exp_name + "_traj.csv")


fig, axs = plt.subplots(2, 2, figsize=(9, 6))
ax_q = axs[0, 0]
ax_tau = axs[1, 0]
ax_scene = axs[0, 1]
ax_fcone = axs[1, 1]
ax_fcone.plot([0, 32], [0, 40], color='k', alpha=0.5)
ax_fcone.plot([0, -32], [0, 40], color='k', alpha=0.5)

ax_q.set_title('joint angle')
ax_tau.set_title('torque')
ax_fcone.set_title('contact force in friction cone')
ax_scene.set_title('trajectory visualization')

ax_q.set_ylabel('$q$ ($^\circ$)')
ax_tau.set_ylabel('$\\tau$ ($N\cdot m$)')
ax_tau.set_xlabel('time ($s$)')
ax_q.set_xlabel('time ($s$)')
ax_fcone.set_xlabel('fx ($N$)')
ax_fcone.set_ylabel('fz ($N$)')
ax_fcone.set_aspect('equal')
ax_scene.set_aspect('equal')
ax_scene.set_xlim([-1.2, 1.2])

rect = patches.Rectangle((-0.5, -0.1), 1, 0.2, linewidth=1, edgecolor='k', facecolor='none')

# Add the patch to the Axes
ax_scene.add_patch(rect)

ax_q.plot(manopt_data["time"], manopt_data["q"] * 180 / np.pi, label="CMC-Opt")
ax_tau.plot(manopt_data["time"], manopt_data["tau"])
ax_fcone.plot(manopt_data["fx"], manopt_data["fy"], linewidth=2)

ax_q.plot(init_data["time"], init_data["q"] * 180 / np.pi, label= "Initial values")
ax_tau.plot(init_data["time"], init_data["tau"])
ax_fcone.scatter(init_data["fx"], init_data["fy"], color='#ff7f0e', zorder=3)
plot_state(ax_scene, manopt_data["q"])

ax_q.legend()

for ax in [ax_q, ax_tau, ax_fcone, ax_scene]:
    ax.title.set_fontsize(16)
    ax.xaxis.label.set_fontsize(14)
    ax.yaxis.label.set_fontsize(14)
    for item in ax.get_xticklabels() + ax.get_yticklabels():
        item.set_fontsize(12)

plt.gcf().text(0.02, 0.94, "(a)", fontsize=14)
plt.gcf().text(0.52, 0.94, "(b)", fontsize=14)
plt.gcf().text(0.02, 0.46, "(c)", fontsize=14)
plt.gcf().text(0.52, 0.46, "(d)", fontsize=14)

plt.tight_layout()
plt.savefig('figures/block_pole_traj.pdf')
plt.show()