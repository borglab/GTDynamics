import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as animation
from yetong00_utils import load_csv

folder = "data/rss04_quad_jump/"


def hurdle(center_x, width, height):
    x = np.linspace(center_x-width/2, center_x+width/2, 100)
    dist_to_center = x - center_x
    rate = np.pi * 2 / width
    theta = dist_to_center * rate
    h = (np.cos(theta) + 1) * height / 2
    return x, h


class QuadPlotter:
    def __init__(self, exp_name):
        self.data = load_csv(folder + exp_name + "_traj.csv")

        self.end_point_names1 = [["fl_hip", "fl_upper"], ["fl_upper", "fl_lower"], ["fl_lower", "fl_lower_c"],
                                 ["rl_hip", "rl_upper"], ["rl_upper", "rl_lower"], [
                                     "rl_lower", "rl_lower_c"],
                                 ["torso_fl_top", "torso_rl_top"], [
                                     "torso_fl_bot", "torso_rl_bot"],
                                 ["torso_fl_top", "torso_fl_bot"], ["torso_rl_top", "torso_rl_bot"]]
        self.end_point_names2 = [["torso_fl_top", "torso_rl_top"], ["torso_fr_top", "torso_rr_top"],
                                 ["torso_fl_top", "torso_fr_top"], [
                                     "torso_rl_top", "torso_rr_top"],
                                 ["fl_hip", "fl_upper"], ["fl_upper", "fl_lower"], [
                                     "fl_lower", "fl_lower_c"],
                                 ["rl_hip", "rl_upper"], ["rl_upper", "rl_lower"], [
                                     "rl_lower", "rl_lower_c"],
                                 ["fr_hip", "fr_upper"], ["fr_upper", "fr_lower"], [
                                     "fr_lower", "fr_lower_c"],
                                 ["rr_hip", "rr_upper"], ["rr_upper", "rr_lower"], ["rr_lower", "rr_lower_c"]]

        self.colors1 = ["k", "r", "orange", "k", "g", "b", "k", "k", "k", "k"]
        self.colors2 = ["k", "k", "k", "k", "k", "r", "orange",
                        "k", "g", "b", "k", "r", "orange", "k", "g", "b"]

        self.joint_names = ["fl_upper", "fl_lower", "rl_upper", "rl_lower"]
        self.joint_colors = ['r', 'orange', 'g', 'b']
        self.contact_names = ["fl_lower_c", "rl_lower_c"]
        self.contact_colors = ['orange', 'b']

        self.step_plots_entry_name_x1 = []
        self.step_plots_entry_name_x2 = []
        self.step_plots_entry_name_y1 = []
        self.step_plots_entry_name_y2 = []
        self.step_plots_color = []

        # info of all step plots
        for (start_pt_name, end_pt_name), color in zip(self.end_point_names1, self.colors1):
            self.step_plots_entry_name_x1 += [start_pt_name+"_x"]
            self.step_plots_entry_name_x2 += [end_pt_name+"_x"]
            self.step_plots_entry_name_y1 += [start_pt_name+"_z"]
            self.step_plots_entry_name_y2 += [end_pt_name+"_z"]
            self.step_plots_color += [color]

    def plot_enviroment(self, ax):
        hurdle_x, hurdle_z = hurdle(0.75, 0.3, 0.2)
        ax.plot(hurdle_x, hurdle_z, color='k')
        ax.plot([-1, 3], [0, 0], color='k', alpha=0.5)

    def plot_step(self, ax, k):
        colormap = mpl.colormaps["jet"]
        boundary_steps = [0, 20, 30, 50, 70]
        for entry_name_x1, entry_name_x2, entry_name_y1, entry_name_y2, color in zip(
                self.step_plots_entry_name_x1,
                self.step_plots_entry_name_x2,
                self.step_plots_entry_name_y1,
                self.step_plots_entry_name_y2,
                self.step_plots_color):
            x1 = self.data[entry_name_x1][k]
            x2 = self.data[entry_name_x2][k]
            y1 = self.data[entry_name_y1][k]
            y2 = self.data[entry_name_y2][k]
            robot_color = colormap(k/70)
            alpha = 1 if k in boundary_steps else 0.7
            linewidth = 3 if k in boundary_steps else 2
            ax.plot((x1, x2), (y1, y2), color=robot_color,
                    alpha=alpha, linewidth=linewidth)

    def plot_quantity(self, ax, entry_name):
        rate = 180/np.pi if entry_name == "_q" else 1
        for joint_name, color in zip(self.joint_names, self.joint_colors):
            ax.plot(self.data["time"], self.data[joint_name+entry_name] * rate,
                    color=color, alpha=1, label=joint_name)

    def plot_contact_force(self, ax):
        for contact_name, color in zip(self.contact_names, self.contact_colors):
            contact_fx = self.data[contact_name+"_fx"]
            contact_fy = self.data[contact_name+"_fy"]
            contact_fz = self.data[contact_name+"_fz"]
            contact_f = (contact_fx**2 + contact_fy**2 + contact_fz**2) ** 0.5
            ax.plot(self.data["time"], contact_f,
                    color=color, alpha=1, label=contact_name)

    def plot_friction_cone(self, ax):
        for contact_name, color in zip(self.contact_names, self.contact_colors):
            ax.plot(self.data[contact_name+"_fx"], self.data[contact_name+"_fz"],
                    color=color, alpha=1, label=contact_name)


def plot_scenario():
    fig, ax = plt.subplots(figsize=(9, 2))
    ax.set_xlim(-1, 2.3)
    ax.set_ylim(-0, 0.5)
    ax.set_aspect('equal')
    quad_plotter = QuadPlotter()

    k_list = [0, 70]
    for k in k_list:
        quad_plotter.plot_step(ax, k)

    hurdle_x, hurdle_z = hurdle(0.75, 0.3, 0.2)
    ax.plot(hurdle_x, hurdle_z, color='k')
    ax.plot([-1, 3], [0, 0], color='k', alpha=0.5)
    ax.set_axis_off()
    plt.tight_layout()

    plt.savefig('figures/quad_scenario.pdf')
    plt.show()


def plot_trajectory_comparisoin():
    fig, axs = plt.subplots(2, 1, figsize=(9, 6))
    ax1 = axs[0]
    ax2 = axs[1]
    for ax in [ax1, ax2]:
        ax.set_xlim(-1, 2.3)
        ax.set_ylim(-0, 1)
        ax.set_aspect('equal')

    quad_plotter_manopt = QuadPlotter("manopt")
    quad_plotter_penalty = QuadPlotter("penalty")

    k_list = [0, 15, 20, 30, 36, 44, 50, 60, 70]

    for ax, quad_plotter in zip([ax1, ax2], [quad_plotter_penalty, quad_plotter_manopt]):
        quad_plotter.plot_enviroment(ax)
        for k in k_list:
            quad_plotter.plot_step(ax, k)
    plt.tight_layout()

    plt.savefig('figures/quad_traj_comp.pdf')

    plt.show()



def plot_trajectory():
    fig, ax = plt.subplots(figsize=(9, 3))
    ax.set_xlim(-1, 2.3)
    ax.set_ylim(-0, 1)
    ax.set_aspect('equal')
    quad_plotter = QuadPlotter()
    quad_plotter.plot_enviroment(ax)

    k_list = [0, 15, 20, 30, 36, 44, 50, 60, 70]
    for k in k_list:
        quad_plotter.plot_step(ax, k)


    plt.tight_layout()

    plt.savefig('figures/quad_traj.pdf')

    plt.show()


def plot_quantities():
    fig, ax = plt.subplots(2, 2, figsize=(9, 6))
    ax_q = ax[0, 0]
    ax_T = ax[1, 0]
    ax_fcone = ax[0, 1]
    ax_cforce = ax[1, 1]

    ax_q.set_title('joint angle')
    ax_T.set_title('torque')
    ax_fcone.set_title('contact force')
    ax_cforce.set_title('contact force')

    ax_q.set_ylabel('$q$ ($^\circ$)')
    ax_T.set_ylabel('$\\tau$ ($N\cdot m$)')
    ax_cforce.set_ylabel('contact force ($N$)')
    ax_T.set_xlabel('time ($s$)')
    ax_cforce.set_xlabel('time ($s$)')
    ax_q.set_xlabel('time ($s$)')
    ax_fcone.set_xlabel('fx ($N$)')
    ax_fcone.set_ylabel('fz ($N$)')

    ax_fcone.set_aspect('equal')

    ax_fcone.plot([0, 100], [0, 100], color='k', alpha=0.5)
    ax_fcone.plot([0, -100], [0, 100], color='k', alpha=0.5)

    quad_plotter = QuadPlotter()
    quad_plotter.plot_quantity(ax_q, "_q")
    quad_plotter.plot_quantity(ax_T, "_T")
    quad_plotter.plot_friction_cone(ax_fcone)
    quad_plotter.plot_contact_force(ax_cforce)
    # quad_plotter.plot_quantity(ax_q, "_q")
    # quad_plotter.plot_quantity(ax_q, "_q")

    ax_q.legend()
    for ax in [ax_q, ax_T, ax_fcone, ax_cforce]:
        ax.title.set_fontsize(16)
        ax.xaxis.label.set_fontsize(14)
        ax.yaxis.label.set_fontsize(14)
        for item in ax.get_xticklabels() + ax.get_yticklabels():
            item.set_fontsize(12)

    plt.tight_layout()
    plt.savefig('figures/quad_quantities.pdf')
    plt.show()


plot_trajectory_comparisoin()
# plot_trajectory()
# plot_quantities()
# plot_scenario()
