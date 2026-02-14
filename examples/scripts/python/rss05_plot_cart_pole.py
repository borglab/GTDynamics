import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches as patches
from yetong00_utils import load_csv

folder = "data/rss02_cp_limits/"
exp_name= "ielm"


def plot_state(ax, x, q):
    colormap = mpl.colormaps["jet"]
    r = 0.3
    for i in range(0, len(q), 1):
        ax.plot([x[i], x[i]+ r * np.sin(q[i])], [0, -r * np.cos(q[i])], color=colormap((len(q)-i)/len(q)))

init_data = load_csv(folder + "init_traj.csv")
manopt_data = load_csv(folder + exp_name + "_traj.csv")

fig, axs = plt.subplots(2, 2, figsize=(9, 6))
ax_q = axs[0, 0]
ax_f = axs[1, 0]
ax_scene = axs[0, 1]
ax_x = axs[1, 1]

ax_q.set_title('joint angle')
ax_f.set_title('force')
ax_x.set_title('cart position')
ax_scene.set_title('trajectory visualization')
ax_scene.set_aspect('equal')

ax_q.plot(manopt_data["t"], manopt_data["theta"] * 180 / np.pi, label="CMC-Opt")
ax_f.plot(manopt_data["t"], manopt_data["xtau"])
ax_x.plot(manopt_data["t"], manopt_data["x"])

ax_q.plot(init_data["t"], init_data["theta"] * 180 / np.pi, label = "Initial values")
ax_f.plot(init_data["t"], init_data["xtau"])
ax_x.plot(init_data["t"], init_data["x"])
ax_f.set_xlabel('time ($s$)')
ax_q.set_xlabel('time ($s$)')
ax_x.set_xlabel('time ($s$)')
ax_q.set_ylabel('$q$ ($^\circ$)')
ax_f.set_ylabel('$f$ ($N$)')
ax_x.set_ylabel('$x$ ($m$)')

ax_x.plot([init_data["t"][0], init_data["t"][-1]], [-0.2, -0.2], color="k", zorder=0)
ax_x.plot([init_data["t"][0], init_data["t"][-1]], [0.2, 0.2], color="k", zorder=0)

ax_f.plot([init_data["t"][0], init_data["t"][-1]], [-100, -100], color="k", zorder=0)
ax_f.plot([init_data["t"][0], init_data["t"][-1]], [100, 100], color="k", zorder=0)

ax_q.legend()

for ax in [ax_q, ax_x, ax_f, ax_scene]:
    ax.title.set_fontsize(16)
    ax.xaxis.label.set_fontsize(14)
    ax.yaxis.label.set_fontsize(14)
    for item in ax.get_xticklabels() + ax.get_yticklabels():
        item.set_fontsize(12)

ax_scene.plot([-0.5, 0.5], [0, 0], color='k', alpha = 0.5, linewidth=3)
# ax_scene.plot([-0.2, -0.2], [-0.4, 0.4], color='k', alpha = 0.5)
# ax_scene.plot([0.2, 0.2], [-0.4, 0.4], color='k', alpha = 0.5)

plot_state(ax_scene, manopt_data["x"], manopt_data["theta"])

plt.gcf().text(0.02, 0.94, "(a)", fontsize=14)
plt.gcf().text(0.52, 0.94, "(b)", fontsize=14)
plt.gcf().text(0.02, 0.46, "(c)", fontsize=14)
plt.gcf().text(0.52, 0.46, "(d)", fontsize=14)

plt.tight_layout()
plt.savefig('figures/cart_pole_traj.pdf')
plt.show()
