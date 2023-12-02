import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as animation
from yetong00_utils import load_csv

folder = "data/yetong07_e_quadruped_jump/"

# data = load_csv(folder + "init_traj.csv")
data= load_csv(folder + "manopt_traj.csv")


class PauseAnimation:
    def __init__(self):
        self.num_steps = len(data["time"])

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
        self.ax_top.set_xlim(-1, 2)
        self.ax_top.set_ylim(-0.5, 0.5)
        self.ax_top.set_aspect('equal')
        self.ax_top.set_xlabel('x')
        self.ax_top.set_ylabel('y')
        self.ax_q.set_xlabel('time')
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
        self.ax_fcone.set_aspect('equal')
        self.ax_fcone.set_xlabel('fx')
        self.ax_fcone.set_ylabel('fz')
        self.ax_fcone.set_title('friction cone')
        self.ax_fcone.plot([0, 100], [0, 100], color='k', alpha=0.5)
        self.ax_fcone.plot([0, -100], [0, 100], color='k', alpha=0.5)

        self.plots_side = []
        self.plots_top = []
        self.plots_jangles1 = []
        self.plots_jangles2 = []
        self.plots_jvels1 = []
        self.plots_jvels2 = []
        self.plots_jaccels1 = []
        self.plots_jaccels2 = []
        self.plots_torques1 = []
        self.plots_torques2 = []
        self.plots_cforce1 = []
        self.plots_cforce2 = []
        self.plots_torso_v1 = []
        self.plots_torso_v2 = []
        self.plots_torso_a1 = []
        self.plots_torso_a2 = []
        self.plots_fcone1 = []
        self.plots_fcone2 = []

        self.end_point_names1 = [["fl_hip", "rl_hip"],
                                ["fl_hip", "fl_upper"], ["fl_upper", "fl_lower"], ["fl_lower", "fl_lower_c"],
                                ["rl_hip", "rl_upper"], ["rl_upper", "rl_lower"], ["rl_lower", "rl_lower_c"]]
        self.end_point_names2 = [["fl_hip", "rl_hip"], ["fr_hip", "rr_hip"], ["fl_hip", "fr_hip"], ["rl_hip", "rr_hip"],
                                ["fl_hip", "fl_upper"], ["fl_upper", "fl_lower"], ["fl_lower", "fl_lower_c"],
                                ["rl_hip", "rl_upper"], ["rl_upper", "rl_lower"], ["rl_lower", "rl_lower_c"],
                                ["fr_hip", "fr_upper"], ["fr_upper", "fr_lower"], ["fr_lower", "fr_lower_c"],
                                ["rr_hip", "rr_upper"], ["rr_upper", "rr_lower"], ["rr_lower", "rr_lower_c"]]

        self.colors1 = ["k", "k", "r", "orange", "k", "g", "b"]
        self.colors2 = ["k", "k","k","k", "k", "r", "orange", "k", "g", "b", "k", "r", "orange", "k", "g", "b"]

        self.joint_names = [ "fl_upper", "fl_lower", "rl_upper", "rl_lower"]
        self.joint_colors = ['r', 'orange', 'g', 'b']

        self.contact_names = [ "fl_lower_c", "rl_lower_c"]
        self.contact_colors = ['orange', 'b']

        # max_q, min_q, max_v, min_v, max_a, min_a, max_T, min_T = 0, 0, 0, 0, 0, 0, 0, 0
        # for joint_name in self.joint_names:
        #     max_q = max(max_q, data[joint_name + "_q"].max())
        #     min_q = min(min_q, data[joint_name + "_q"].min())
        #     max_v = max(max_v, data[joint_name + "_v"].max())
        #     min_v = min(min_v, data[joint_name + "_v"].min())
        #     max_a = max(max_a, data[joint_name + "_a"].max())
        #     min_a = min(min_a, data[joint_name + "_a"].min())
        #     max_T = max(max_T, data[joint_name + "_T"].max())
        #     min_T = min(min_T, data[joint_name + "_T"].min())
        # self.ax_q.set_ylim(min_q, max_q)
        # self.ax_v.set_ylim(min_v, max_v)
        # self.ax_a.set_ylim(min_a, max_a)
        # self.ax_T.set_ylim(min_T, max_T)

        k = 0
        for (start_pt_name, end_pt_name), color in zip(self.end_point_names1, self.colors1):
            x1 = data[start_pt_name+"_x"][k]
            y1 = data[start_pt_name+"_y"][k]
            z1 = data[start_pt_name+"_z"][k]
            x2 = data[end_pt_name+"_x"][k]
            y2 = data[end_pt_name+"_y"][k]
            z2 = data[end_pt_name+"_z"][k]

            frame = self.ax_side.plot((x1, x2), (z1, z2),
                                    color=color, alpha=0.5, animated=True)[0]
            self.plots_side.append(frame)

        for (start_pt_name, end_pt_name), color in zip(self.end_point_names2, self.colors2):
            x1 = data[start_pt_name+"_x"][k]
            y1 = data[start_pt_name+"_y"][k]
            z1 = data[start_pt_name+"_z"][k]
            x2 = data[end_pt_name+"_x"][k]
            y2 = data[end_pt_name+"_y"][k]
            z2 = data[end_pt_name+"_z"][k]

            frame = self.ax_top.plot((x1, x2), (y1, y2),
                                    color=color, alpha=0.5, animated=True)[0]
            self.plots_top.append(frame)

        for joint_name, color in zip(self.joint_names, self.joint_colors):
            frame = self.ax_q.plot(data["time"][:1], data[joint_name+"_q"][:1],
                         color = color, alpha=1, animated=True, label=joint_name)[0]
            self.plots_jangles1.append(frame)
            frame = self.ax_q.plot(data["time"][1:], data[joint_name+"_q"][1:],
                         color = color, alpha=0.2, animated=True)[0]
            self.plots_jangles2.append(frame)
            frame = self.ax_v.plot(data["time"][:1], data[joint_name+"_v"][:1],
                         color = color, alpha=1, animated=True)[0]
            self.plots_jvels1.append(frame)
            frame = self.ax_v.plot(data["time"][1:], data[joint_name+"_v"][1:],
                         color = color, alpha=0.2, animated=True)[0]
            self.plots_jvels2.append(frame)
            frame = self.ax_a.plot(data["time"][:1], data[joint_name+"_a"][:1],
                         color = color, alpha=1, animated=True)[0]
            self.plots_jaccels1.append(frame)
            frame = self.ax_a.plot(data["time"][1:], data[joint_name+"_a"][1:],
                         color = color, alpha=0.2, animated=True)[0]
            self.plots_jaccels2.append(frame)
            frame = self.ax_T.plot(data["time"][:1], data[joint_name+"_T"][:1],
                         color = color, alpha=1, animated=True)[0]
            self.plots_torques1.append(frame)
            frame = self.ax_T.plot(data["time"][1:], data[joint_name+"_T"][1:],
                         color = color, alpha=0.2, animated=True)[0]
            self.plots_torques2.append(frame)
        self.ax_q.legend()

        self.title_k = self.ax_side.text(0.5, 1.2, str(k), bbox={
                          'facecolor': 'w', 'alpha': 0.5, 'pad': 5}, ha="center", animated=True)

        for contact_name, color in zip(self.contact_names, self.contact_colors):
            frame = self.ax_cforce.plot(data["time"][:1], data[contact_name+"_fz"][:1],
                                        color=color, alpha=1, animated=True)[0]
            self.plots_cforce1.append(frame)
            frame = self.ax_cforce.plot(data["time"][1:], data[contact_name+"_fz"][1:],
                                        color=color, alpha=0.2, animated=True)[0]
            self.plots_cforce2.append(frame)
            frame = self.ax_fcone.plot(data[contact_name+"_fx"][:1], data[contact_name+"_fz"][:1],
                                       color=color, alpha=1, animated=True)[0]
            self.plots_fcone1.append(frame)
            frame = self.ax_fcone.plot(data[contact_name+"_fx"][1:], data[contact_name+"_fz"][1:],
                                       color=color, alpha=0.2, animated=True)[0]
            self.plots_fcone2.append(frame)

        frame = self.ax_torso_v.plot(data["time"][:1], data["base_vz"][:1],
                                     color='b', alpha=1, animated=True)[0]
        self.plots_torso_v1.append(frame)
        frame = self.ax_torso_v.plot(data["time"][1:], data["base_vz"][1:],
                                     color='b', alpha=0.2, animated=True)[0]
        self.plots_torso_v2.append(frame)

        frame = self.ax_torso_a.plot(data["time"][:1], data["base_az"][:1],
                                     color='r', alpha=1, animated=True)[0]
        self.plots_torso_a1.append(frame)
        frame = self.ax_torso_a.plot(data["time"][1:], data["base_az"][1:],
                                     color='r', alpha=0.2, animated=True)[0]
        self.plots_torso_a2.append(frame)

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
        plots += self.plots_side
        plots += self.plots_top
        plots += self.plots_jangles1
        plots += self.plots_jangles2
        plots += self.plots_jvels1
        plots += self.plots_jvels2
        plots += self.plots_jaccels1
        plots += self.plots_jaccels2
        plots += self.plots_torques1
        plots += self.plots_torques2
        plots += self.plots_cforce1
        plots += self.plots_cforce2
        plots += self.plots_torso_v1
        plots += self.plots_torso_v2
        plots += self.plots_torso_a1
        plots += self.plots_torso_a2
        plots += self.plots_fcone1
        plots += self.plots_fcone2
        return plots

    def update(self, k):

        for (start_pt_name, end_pt_name), plot1 in zip(self.end_point_names1, self.plots_side):
            x1 = data[start_pt_name+"_x"][k]
            y1 = data[start_pt_name+"_y"][k]
            z1 = data[start_pt_name+"_z"][k]
            x2 = data[end_pt_name+"_x"][k]
            y2 = data[end_pt_name+"_y"][k]
            z2 = data[end_pt_name+"_z"][k]
            plot1.set_xdata([x1, x2])
            plot1.set_ydata([z1, z2])

        for (start_pt_name, end_pt_name), plot2 in zip(self.end_point_names2, self.plots_top):
            x1 = data[start_pt_name+"_x"][k]
            y1 = data[start_pt_name+"_y"][k]
            z1 = data[start_pt_name+"_z"][k]
            x2 = data[end_pt_name+"_x"][k]
            y2 = data[end_pt_name+"_y"][k]
            z2 = data[end_pt_name+"_z"][k]
            plot2.set_xdata([x1, x2])
            plot2.set_ydata([y1, y2])

        for joint_name, plot1, plot2 in zip(self.joint_names, self.plots_jangles1, self.plots_jangles2):
            plot1.set_xdata(data["time"][:k+1])
            plot1.set_ydata(data[joint_name+"_q"][:k+1])
            plot2.set_xdata(data["time"][k:])
            plot2.set_ydata(data[joint_name+"_q"][k:])

        for joint_name, plot1, plot2 in zip(self.joint_names, self.plots_jvels1, self.plots_jvels2):
            plot1.set_xdata(data["time"][:k+1])
            plot1.set_ydata(data[joint_name+"_v"][:k+1])
            plot2.set_xdata(data["time"][k:])
            plot2.set_ydata(data[joint_name+"_v"][k:])

        for joint_name, plot1, plot2 in zip(self.joint_names, self.plots_jaccels1, self.plots_jaccels2):
            plot1.set_xdata(data["time"][:k+1])
            plot1.set_ydata(data[joint_name+"_a"][:k+1])
            plot2.set_xdata(data["time"][k:])
            plot2.set_ydata(data[joint_name+"_a"][k:])

        for joint_name, plot1, plot2 in zip(self.joint_names, self.plots_torques1, self.plots_torques2):
            plot1.set_xdata(data["time"][:k+1])
            plot1.set_ydata(data[joint_name+"_T"][:k+1])
            plot2.set_xdata(data["time"][k:])
            plot2.set_ydata(data[joint_name+"_T"][k:])


        for contact_name, plot1, plot2 in zip(self.contact_names, self.plots_cforce1, self.plots_cforce2):
            plot1.set_xdata(data["time"][:k+1])
            plot1.set_ydata(data[contact_name+"_fz"][:k+1])
            plot2.set_xdata(data["time"][k:])
            plot2.set_ydata(data[contact_name+"_fz"][k:])

        for contact_name, plot1, plot2 in zip(self.contact_names, self.plots_fcone1, self.plots_fcone2):
            plot1.set_xdata(data[contact_name+"_fx"][:k+1])
            plot1.set_ydata(data[contact_name+"_fz"][:k+1])
            plot2.set_xdata(data[contact_name+"_fx"][k:])
            plot2.set_ydata(data[contact_name+"_fz"][k:])

        for plot1, plot2 in zip(self.plots_torso_v1, self.plots_torso_v2):
            plot1.set_xdata(data["time"][:k+1])
            plot1.set_ydata(data["base_vz"][:k+1])
            plot2.set_xdata(data["time"][k:])
            plot2.set_ydata(data["base_vz"][k:])
        for plot1, plot2 in zip(self.plots_torso_a1, self.plots_torso_a2):
            plot1.set_xdata(data["time"][:k+1])
            plot1.set_ydata(data["base_az"][:k+1])
            plot2.set_xdata(data["time"][k:])
            plot2.set_ydata(data["base_az"][k:])

        self.title_k.set_text(str(k))

        return self.get_all_plots()
        # self.plots.append(frame)

pa = PauseAnimation()
plt.show()
