import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as animation
from yetong00_utils import load_csv

# folder = "data/yetong10_ie_quadruped_jump/"
folder = "data/yetong11_ie_quadruped_jump_land/"

# data = load_csv(folder + "init_traj.csv")
data= load_csv(folder + "manopt_traj.csv")

def hurdle(center_x, width, height):
    x = np.linspace(center_x-width/2, center_x+width/2, 100)
    dist_to_center = x - center_x
    rate = np.pi * 2 / width
    theta = dist_to_center * rate
    h = (np.cos(theta) + 1) * height / 2
    return x, h

class PauseAnimation:
    def __init__(self):
        self.num_steps = len(data["time"])

        # set subplots and ax
        self.fig, self.ax = plt.subplots(3, 3, figsize=(14, 9))
        self.ax_side = self.ax[0, 0]
        self.ax_top = self.ax[0, 1]
        self.ax_q = self.ax[1, 0]
        self.ax_v = self.ax[1, 1]
        self.ax_a = self.ax[2, 0]
        self.ax_T = self.ax[2, 1]
        self.ax_fcone = self.ax[0, 2]
        self.ax_cforce = self.ax[1, 2]
        self.ax_torso_v = self.ax[2, 2]
        self.ax_torso_a = self.ax_torso_v.twinx()
        self.ax_side.set_xlim(-1, 2)
        self.ax_side.set_ylim(-0.5, 1)
        self.ax_side.set_aspect('equal')
        self.ax_side.set_xlabel('x')
        self.ax_side.set_ylabel('z')
        self.ax_side.set_title('side view')
        self.ax_top.set_xlim(-1, 2)
        self.ax_top.set_ylim(-0.5, 0.5)
        self.ax_top.set_aspect('equal')
        self.ax_top.set_xlabel('x')
        self.ax_top.set_ylabel('y')
        self.ax_top.set_title('top view')
        self.ax_q.set_ylabel('joint angle')
        self.ax_q.set_xlim(0, data["time"][-1])
        self.ax_v.set_xlabel('time')
        self.ax_v.set_ylabel('joint vel')
        self.ax_v.set_xlim(0, data["time"][-1])
        self.ax_a.set_xlabel('time')
        self.ax_a.set_ylabel('joint accel')
        self.ax_a.set_xlim(0, data["time"][-1])
        self.ax_T.set_xlabel('time')
        self.ax_T.set_ylabel('torque')
        self.ax_T.set_xlim(0, data["time"][-1])
        self.ax_side.plot([-1, 3], [0, 0], color='k', alpha=0.5)
        self.ax_cforce.set_xlim(0, data["time"][-1])
        self.ax_cforce.set_ylabel('contact force')
        self.ax_torso_v.set_xlim(0, data["time"][-1])
        self.ax_torso_v.set_xlabel('time')
        self.ax_torso_v.set_ylabel('torso vz')
        self.ax_torso_a.set_ylabel('torso az')
        self.ax_torso_v.tick_params(axis='y', labelcolor='b')
        self.ax_torso_a.tick_params(axis='y', labelcolor='r')
        self.ax_torso_v.yaxis.label.set_color('b')
        self.ax_torso_a.yaxis.label.set_color('r')
        self.ax_fcone.set_aspect('equal')
        self.ax_fcone.set_xlabel('fx')
        self.ax_fcone.set_ylabel('fz')
        self.ax_fcone.set_title('friction cone')
        self.ax_fcone.plot([0, 100], [0, 100], color='k', alpha=0.5)
        self.ax_fcone.plot([0, -100], [0, 100], color='k', alpha=0.5)
        hurdle_x, hurdle_z = hurdle(0.75, 0.3, 0.2)
        self.ax_side.plot(hurdle_x, hurdle_z, color='k')

        # general info
        self.end_point_names1 = [["fl_hip", "fl_upper"], ["fl_upper", "fl_lower"], ["fl_lower", "fl_lower_c"],
                                 ["rl_hip", "rl_upper"], ["rl_upper", "rl_lower"], ["rl_lower", "rl_lower_c"],
                                 ["torso_fl_top", "torso_rl_top"], ["torso_fl_bot", "torso_rl_bot"],
                                 ["torso_fl_top", "torso_fl_bot"], ["torso_rl_top", "torso_rl_bot"]]
        self.end_point_names2 = [["torso_fl_top", "torso_rl_top"], ["torso_fr_top", "torso_rr_top"], 
                                 ["torso_fl_top", "torso_fr_top"], ["torso_rl_top", "torso_rr_top"],
                                 ["fl_hip", "fl_upper"], ["fl_upper", "fl_lower"], ["fl_lower", "fl_lower_c"],
                                 ["rl_hip", "rl_upper"], ["rl_upper", "rl_lower"], ["rl_lower", "rl_lower_c"],
                                 ["fr_hip", "fr_upper"], ["fr_upper", "fr_lower"], ["fr_lower", "fr_lower_c"],
                                 ["rr_hip", "rr_upper"], ["rr_upper", "rr_lower"], ["rr_lower", "rr_lower_c"]]

        self.colors1 = ["k", "r", "orange", "k", "g", "b", "k", "k", "k", "k"]
        self.colors2 = ["k", "k", "k", "k", "k", "r", "orange",
                        "k", "g", "b", "k", "r", "orange", "k", "g", "b"]

        self.joint_names = ["fl_upper", "fl_lower", "rl_upper", "rl_lower"]
        self.joint_colors = ['r', 'orange', 'g', 'b']

        self.contact_names = ["fl_lower_c", "rl_lower_c"]
        self.contact_colors = ['orange', 'b']

        self.time_plots_ax = []
        self.time_plots_entry_name_x = []
        self.time_plots_entry_name_y = []
        self.time_plots_color = []
        self.time_plots_label = []
        self.time_plots1 = []
        self.time_plots2 = []

        self.step_plots_entry_name_x1 = []
        self.step_plots_entry_name_x2 = []
        self.step_plots_entry_name_y1 = []
        self.step_plots_entry_name_y2 = []
        self.step_plots_ax = []
        self.step_plots_color = []
        self.step_plots = []

        # info of all step plots
        for (start_pt_name, end_pt_name), color in zip(self.end_point_names1, self.colors1):
            self.step_plots_ax += [self.ax_side]
            self.step_plots_entry_name_x1 += [start_pt_name+"_x"]
            self.step_plots_entry_name_x2 += [end_pt_name+"_x"]
            self.step_plots_entry_name_y1 += [start_pt_name+"_z"]
            self.step_plots_entry_name_y2 += [end_pt_name+"_z"]
            self.step_plots_color += [color]
        for (start_pt_name, end_pt_name), color in zip(self.end_point_names2, self.colors2):
            self.step_plots_ax += [self.ax_top]
            self.step_plots_entry_name_x1 += [start_pt_name+"_x"]
            self.step_plots_entry_name_x2 += [end_pt_name+"_x"]
            self.step_plots_entry_name_y1 += [start_pt_name+"_y"]
            self.step_plots_entry_name_y2 += [end_pt_name+"_y"]
            self.step_plots_color += [color]

        # info of all timed plots
        for joint_name, color in zip(self.joint_names, self.joint_colors):
            self.time_plots_ax += [self.ax_q, self.ax_v, self.ax_a, self.ax_T]
            self.time_plots_entry_name_x += ["time", "time", "time", "time"]
            self.time_plots_entry_name_y += [joint_name+"_q",
                                             joint_name+"_v", joint_name+"_a", joint_name+"_T"]
            self.time_plots_color += [color, color, color, color]
            self.time_plots_label += [joint_name,
                                      joint_name, joint_name, joint_name]
        for contact_name, color in zip(self.contact_names, self.contact_colors):
            self.time_plots_ax += [self.ax_cforce, self.ax_fcone]
            self.time_plots_entry_name_x += ["time", contact_name+"_fx"]
            self.time_plots_entry_name_y += [contact_name +
                                             "_fz", contact_name+"_fz"]
            self.time_plots_color += [color, color]
            self.time_plots_label += [contact_name, contact_name]
        self.time_plots_ax += [self.ax_torso_v, self.ax_torso_a]
        self.time_plots_entry_name_x += ["time", "time"]
        self.time_plots_entry_name_y += ["base_vz", "base_az"]
        self.time_plots_color += ['b', 'r']
        self.time_plots_label += ["torso_vz", "torso_az"]

        # make time plots
        for ax, entry_name_x, entry_name_y, color, label, in zip(self.time_plots_ax,
                                                                 self.time_plots_entry_name_x,
                                                                 self.time_plots_entry_name_y,
                                                                 self.time_plots_color,
                                                                 self.time_plots_label):
            frame = ax.plot(data[entry_name_x][:1], data[entry_name_y][:1],
                            color=color, alpha=1, animated=True, label=label)[0]
            self.time_plots1.append(frame)
            frame = ax.plot(data[entry_name_x][1:], data[entry_name_y][1:],
                            color=color, alpha=0.2, animated=True)[0]
            self.time_plots2.append(frame)
            self.time_plots1
        self.ax_q.legend()

        # make step plots
        for ax, entry_name_x1, entry_name_x2, entry_name_y1, entry_name_y2, color in zip(self.step_plots_ax,
                                                                                         self.step_plots_entry_name_x1,
                                                                                         self.step_plots_entry_name_x2,
                                                                                         self.step_plots_entry_name_y1,
                                                                                         self.step_plots_entry_name_y2,
                                                                                         self.step_plots_color):
            x1 = data[entry_name_x1][0]
            x2 = data[entry_name_x2][0]
            y1 = data[entry_name_y1][0]
            y2 = data[entry_name_y2][0]
            frame = ax.plot((x1, x2), (y1, y2),
                            color=color, alpha=0.5, animated=True)[0]
            self.step_plots.append(frame)

        self.title_k = self.ax_side.text(0.5, 1.5, str(0), bbox={
            'facecolor': 'w', 'alpha': 0.5, 'pad': 5}, ha="center", animated=True)

        self.animation = animation.FuncAnimation(
            self.fig, self.update, frames=self.num_steps, interval=300, blit=True)
        self.paused = False

        self.fig.canvas.mpl_connect('button_press_event', self.toggle_pause)

    def toggle_pause(self, *args, **kwargs):
        if self.paused:
            self.animation.resume()
        else:
            self.animation.pause()
        self.paused = not self.paused

    def get_all_plots(self):
        plots = []
        plots.append(self.title_k)
        plots += self.step_plots
        plots += self.time_plots1
        plots += self.time_plots2
        return plots

    def update(self, k):
        # update time plots
        for plot, entry_name_x1, entry_name_x2, entry_name_y1, entry_name_y2 in zip(self.step_plots,
                                                                                    self.step_plots_entry_name_x1,
                                                                                    self.step_plots_entry_name_x2,
                                                                                    self.step_plots_entry_name_y1,
                                                                                    self.step_plots_entry_name_y2):
            x1 = data[entry_name_x1][k]
            x2 = data[entry_name_x2][k]
            y1 = data[entry_name_y1][k]
            y2 = data[entry_name_y2][k]
            plot.set_xdata([x1, x2])
            plot.set_ydata([y1, y2])

        # update step plots
        for plot1, plot2, entry_name_x, entry_name_y in zip(self.time_plots1,
                                                            self.time_plots2,
                                                            self.time_plots_entry_name_x,
                                                            self.time_plots_entry_name_y):
            plot1.set_xdata(data[entry_name_x][:k+1])
            plot1.set_ydata(data[entry_name_y][:k+1])
            plot2.set_xdata(data[entry_name_x][k:])
            plot2.set_ydata(data[entry_name_y][k:])

        # update time step info
        self.title_k.set_text(str(k))

        return self.get_all_plots()


pa = PauseAnimation()
plt.show()
